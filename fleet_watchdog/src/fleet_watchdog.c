#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <signal.h>

#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "fleet_watchdog_client.h"

#define FW_TIMER_PULSE       30
#define FW_CHECK_INTERVAL_MS 500
#define FW_TIMEOUT_MS        2500
#define FW_STARTUP_GRACE_MS  10000

#define FW_PROCESS_COUNT 3

typedef struct
{
    int role;
    const char *name;

    int seen;
    int fault_reported;

    int process_id;

    uint32_t last_sequence;
    uint64_t last_heartbeat_ms;

} fw_process_state;

static fw_process_state processes[FW_PROCESS_COUNT] =
{
    { FW_ROLE_MANAGER, "Coordination Manager", 0, 0, -1, 0, 0 },
    { FW_ROLE_ROVER_1, "Rover 1",              0, 0, -1, 0, 0 },
    { FW_ROLE_ROVER_2, "Rover 2",              0, 0, -1, 0, 0 }
};

static timer_t fw_timer;
static int fw_timer_initialized = 0;
static int fw_coid = -1;

static uint64_t startup_time_ms = 0;

/* Get monotonic time */
static uint64_t get_time_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1)
    {
        perror("clock_gettime");
        return 0;
    }

    return ((uint64_t)ts.tv_sec * 1000ULL) +
           ((uint64_t)ts.tv_nsec / 1000000ULL);
}

/* Find monitored process */
static int find_process(int role)
{
    int i;

    for (i = 0; i < FW_PROCESS_COUNT; i++)
    {
        if (processes[i].role == role)
            return i;
    }

    return -1;
}

/* Process incoming heartbeat */
static void process_heartbeat(
    int rcvid,
    const fw_heartbeat_message *msg)
{
    fw_heartbeat_reply reply;

    int index;
    uint64_t now;

    reply.status = EINVAL;

    if (msg->message_type != FW_MSG_HEARTBEAT)
    {
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
        return;
    }

    index = find_process(msg->role);

    if (index == -1)
    {
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
        return;
    }

    now = get_time_ms();

    if (now == 0 ||
        msg->timestamp_ms == 0 ||
        msg->timestamp_ms > now ||
        now - msg->timestamp_ms > FW_TIMEOUT_MS ||
        msg->process_id <= 0)
    {
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
        return;
    }

    if (processes[index].seen)
    {
        if (msg->process_id != processes[index].process_id ||
            msg->sequence <= processes[index].last_sequence)
        {
            MsgReply(rcvid, EOK, &reply, sizeof(reply));
            return;
        }
    }

    processes[index].seen = 1;
    processes[index].fault_reported = 0;

    processes[index].process_id = msg->process_id;
    processes[index].last_sequence = msg->sequence;
    processes[index].last_heartbeat_ms = now;

    reply.status = FW_ACK_OK;

    MsgReply(rcvid, EOK, &reply, sizeof(reply));

    printf(
        "[FW HEARTBEAT] %-22s PID=%d Sequence=%u\n",
        processes[index].name,
        processes[index].process_id,
        processes[index].last_sequence
    );
}

/* Check heartbeat timeout */
static void check_process_health(void)
{
    uint64_t now = get_time_ms();
    uint64_t elapsed;

    int i;

    if (now == 0)
        return;

    for (i = 0; i < FW_PROCESS_COUNT; i++)
    {
        if (!processes[i].seen)
        {
            elapsed = now - startup_time_ms;

            if (elapsed >= FW_STARTUP_GRACE_MS &&
                !processes[i].fault_reported)
            {
                printf(
                    "\n[FLEET WATCHDOG FAULT] %s has not sent its first heartbeat\n",
                    processes[i].name
                );

                processes[i].fault_reported = 1;
            }

            continue;
        }

        elapsed = now - processes[i].last_heartbeat_ms;

        if (elapsed > FW_TIMEOUT_MS)
        {
            if (!processes[i].fault_reported)
            {
                printf("\n====================================\n");
                printf("[FLEET WATCHDOG] PROCESS FAILURE\n");
                printf("Process: %s\n", processes[i].name);
                printf("PID: %d\n", processes[i].process_id);
                printf("Heartbeat age: %llu ms\n",
                       (unsigned long long)elapsed);
                printf("====================================\n");

                processes[i].fault_reported = 1;
            }
        }
        else if (processes[i].fault_reported)
        {
            printf(
                "[FLEET WATCHDOG] %s heartbeat restored\n",
                processes[i].name
            );

            processes[i].fault_reported = 0;
        }
    }
}

/* Initialize periodic timer */
static int initialize_timer(int chid)
{
    struct sigevent event;
    struct _itimer timer_spec;

    fw_coid = ConnectAttach(
        ND_LOCAL_NODE,
        0,
        chid,
        _NTO_SIDE_CHANNEL,
        0
    );

    if (fw_coid == -1)
    {
        perror("Fleet Watchdog ConnectAttach");
        return -1;
    }

    SIGEV_PULSE_INIT(
        &event,
        fw_coid,
        SIGEV_PULSE_PRIO_INHERIT,
        FW_TIMER_PULSE,
        0
    );

    fw_timer = TimerCreate(CLOCK_MONOTONIC, &event);

    if (fw_timer == -1)
    {
        perror("Fleet Watchdog TimerCreate");
        ConnectDetach(fw_coid);
        fw_coid = -1;
        return -1;
    }

    memset(&timer_spec, 0, sizeof(timer_spec));

    timer_spec.nsec =
        (long long)FW_CHECK_INTERVAL_MS * 1000000LL;

    timer_spec.interval_nsec =
        (long long)FW_CHECK_INTERVAL_MS * 1000000LL;

    if (TimerSettime(fw_timer, 0, &timer_spec, NULL) == -1)
    {
        perror("Fleet Watchdog TimerSettime");

        TimerDestroy(fw_timer);
        ConnectDetach(fw_coid);

        fw_coid = -1;
        return -1;
    }

    fw_timer_initialized = 1;

    return 0;
}

/* Cleanup */
static void cleanup(name_attach_t *attach)
{
    if (fw_timer_initialized)
    {
        TimerDestroy(fw_timer);
        fw_timer_initialized = 0;
    }

    if (fw_coid != -1)
    {
        ConnectDetach(fw_coid);
        fw_coid = -1;
    }

    if (attach != NULL)
        name_detach(attach, 0);
}

/* Main */
int main(void)
{
    name_attach_t *attach;
    rcvid_t rcvid;

    union
    {
        struct _pulse pulse;
        fw_heartbeat_message heartbeat;
    } received;

    printf("\n====================================\n");
    printf("       FLEET WATCHDOG STARTED\n");
    printf("====================================\n");

    startup_time_ms = get_time_ms();

    if (startup_time_ms == 0)
        return EXIT_FAILURE;

    attach = name_attach(NULL, FW_SERVER_NAME, 0);

    if (attach == NULL)
    {
        perror("Fleet Watchdog name_attach");
        return EXIT_FAILURE;
    }

    printf(
        "[FLEET WATCHDOG] Registered: %s\n",
        FW_SERVER_NAME
    );

    if (initialize_timer(attach->chid) == -1)
    {
        cleanup(attach);
        return EXIT_FAILURE;
    }

    printf("[FLEET WATCHDOG] Monitoring started\n");
    printf("[FLEET WATCHDOG] Timeout: %d ms\n", FW_TIMEOUT_MS);

    while (1)
    {
        rcvid = MsgReceive(
            attach->chid,
            &received,
            sizeof(received),
            NULL
        );

        if (rcvid == -1)
        {
            if (errno == EINTR)
                continue;

            perror("Fleet Watchdog MsgReceive");
            break;
        }

        if (rcvid == 0)
        {
            if (received.pulse.code == FW_TIMER_PULSE)
                check_process_health();

            continue;
        }

        process_heartbeat(rcvid, &received.heartbeat);
    }

    cleanup(attach);

    return EXIT_SUCCESS;
}
