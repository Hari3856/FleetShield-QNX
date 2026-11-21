#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <process.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include "fleet_common.h" // both rovers and manager must share the same definitions and layout
#include "fleet_watchdog_client.h"
#define MANAGER_TIMER_PULSE 10 // configuration
#define MANAGER_TIMER_MS 1000
#define MAX_MESSAGE_AGE_MS 3000ULL
#define ROBOT_COUNT_LOCAL 2
#define MANAGER_PRIORITY 20
#define ROBOT_EXECUTABLE "/tmp/robot_bin/rover"
#define MAX_SEGMENTS 10
#define MAX_WAITING_ROBOTS ROBOT_COUNT
#define MANAGER_SAFETY_NORMAL 0 // manager-specific safety states kept separate from rover watchdog
#define MANAGER_SAFETY_DEGRADED 1
#define MANAGER_SAFETY_SAFE_STOP 2
#define MANAGER_SAFETY_RECOVERY 3
#define MANAGER_SAFETY_EMERGENCY_STOP 4
static timer_t manager_timer; // global variables
static int manager_coid = -1;
static int timer_initialized = 0;
static pthread_mutex_t robot_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t reservation_mutex = PTHREAD_MUTEX_INITIALIZER;
static int fw_manager_coid = -1; // fleet watchdog client connection
static uint32_t fw_manager_sequence = 0;
static int fleet_safety_state = MANAGER_SAFETY_NORMAL; // fleet safety information
static int emergency_stop_active = 0;
typedef struct { // robot state structure
    int active;
    int seen;
    uint32_t last_sequence;
    uint32_t last_heartbeat;
    int status;
    uint64_t last_timestamp_ms;
    uint64_t last_receive_ms;
} robot_state;
static robot_state robots[ROBOT_COUNT_LOCAL] = {0};
typedef struct { // segment reservation structures
    int robot_id;
    int priority;
    uint64_t request_timestamp_ms;
} waiting_request;
typedef struct {
    int segment_id;
    int state;
    int owner_robot_id;
    int waiter_count;
    waiting_request waiting_queue[MAX_WAITING_ROBOTS];
} segment_reservation;
static segment_reservation reservation_table[MAX_SEGMENTS];
static uint64_t get_time_ms(void) { // time function
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1) {
        perror("clock_gettime");
        return 0;
    }
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}
static void initialize_reservation_table(void) { // initialize reservation table
    int i, j;
    for (i = 0; i < MAX_SEGMENTS; i++) {
        reservation_table[i].segment_id = -1;
        reservation_table[i].state = SEGMENT_FREE;
        reservation_table[i].owner_robot_id = 0;
        reservation_table[i].waiter_count = 0;
        for (j = 0; j < MAX_WAITING_ROBOTS; j++) {
            reservation_table[i].waiting_queue[j].robot_id = 0;
            reservation_table[i].waiting_queue[j].priority = 0;
            reservation_table[i].waiting_queue[j].request_timestamp_ms = 0;
        }
    }
    printf("[RESERVATION] Reservation table initialized\n");
}
static int find_segment_slot(int segment_id) { // find segment slot
    int i;
    for (i = 0; i < MAX_SEGMENTS; i++) {
        if (reservation_table[i].segment_id == segment_id) return i;
    }
    return -1;
}
static int request_segment_reservation(int robot_id, int segment_id, int priority) { // returns 1 if granted, 0 if queued, -1 if invalid or full
    uint64_t request_time = get_time_ms();
    int slot = -1;
    int free_slot = -1;
    int i;
    if ((robot_id != 1 && robot_id != 2) || segment_id <= 0 || priority < 0 || request_time == 0) {
        return -1;
    }
    pthread_mutex_lock(&reservation_mutex);
    for (i = 0; i < MAX_SEGMENTS; i++) { // search for existing segment or free slot
        if (reservation_table[i].segment_id == segment_id) {
            slot = i;
            break;
        }
        if (reservation_table[i].segment_id == -1 && reservation_table[i].owner_robot_id == 0 && reservation_table[i].waiter_count == 0 && free_slot == -1) {
            free_slot = i;
        }
    }
    if (slot == -1) { // create new segment entry
        if (free_slot == -1) {
            pthread_mutex_unlock(&reservation_mutex);
            return -1;
        }
        slot = free_slot;
        reservation_table[slot].segment_id = segment_id;
        reservation_table[slot].state = SEGMENT_FREE;
        reservation_table[slot].owner_robot_id = 0;
        reservation_table[slot].waiter_count = 0;
    }
    segment_reservation *segment = &reservation_table[slot];
    if (segment->owner_robot_id == robot_id) { // existing owner retains reservation
        pthread_mutex_unlock(&reservation_mutex);
        return 1;
    }
    for (i = 0; i < segment->waiter_count; i++) { // avoid duplicate waiting entries
        if (segment->waiting_queue[i].robot_id == robot_id) {
            pthread_mutex_unlock(&reservation_mutex);
            return 0;
        }
    }
    if (segment->state == SEGMENT_FREE && segment->owner_robot_id == 0 && segment->waiter_count == 0) { // grant immediately if free
        segment->owner_robot_id = robot_id;
        segment->state = SEGMENT_RESERVED;
        printf("[RESERVATION] Segment %d granted to Robot %d\n", segment_id, robot_id);
        pthread_mutex_unlock(&reservation_mutex);
        return 1;
    }
    if (segment->waiter_count >= MAX_WAITING_ROBOTS) { // check waiting queue capacity
        pthread_mutex_unlock(&reservation_mutex);
        return -1;
    }
    int pos = segment->waiter_count; // add robot to waiting queue
    segment->waiting_queue[pos].robot_id = robot_id;
    segment->waiting_queue[pos].priority = priority;
    segment->waiting_queue[pos].request_timestamp_ms = request_time;
    segment->waiter_count++;
    for (i = segment->waiter_count - 1; i > 0; i--) { // sort by higher priority first, then earlier timestamp on ties
        waiting_request *current = &segment->waiting_queue[i];
        waiting_request *previous = &segment->waiting_queue[i - 1];
        int should_swap = (current->priority > previous->priority) || (current->priority == previous->priority && current->request_timestamp_ms < previous->request_timestamp_ms);
        if (!should_swap) break;
        waiting_request temp = *current;
        *current = *previous;
        *previous = temp;
    }
    printf("[RESERVATION] Robot %d queued for Segment %d\n", robot_id, segment_id);
    pthread_mutex_unlock(&reservation_mutex);
    return 0;
}
static void process_segment_status(const robot_message *msg) { // release segment when owner reports clear and promote next queued robot
    int slot;
    int j;
    if (msg->segment_id <= 0) return;
    pthread_mutex_lock(&reservation_mutex);
    slot = find_segment_slot(msg->segment_id);
    if (slot < 0) {
        pthread_mutex_unlock(&reservation_mutex);
        return;
    }
    segment_reservation *seg = &reservation_table[slot];
    if (msg->segment_occupied && seg->owner_robot_id == msg->robot_id) { // only the owner can mark its segment occupied
        seg->state = SEGMENT_OCCUPIED;
        printf("[RESERVATION] Robot %d occupies Segment %d\n", msg->robot_id, msg->segment_id);
    }
    if (msg->segment_clear && seg->owner_robot_id == msg->robot_id) { // only the owner can release its segment
        printf("[RESERVATION] Robot %d cleared Segment %d\n", msg->robot_id, msg->segment_id);
        seg->owner_robot_id = 0;
        seg->state = SEGMENT_FREE;
        if (seg->waiter_count > 0) { // promote next waiting robot
            waiting_request next = seg->waiting_queue[0];
            for (j = 1; j < seg->waiter_count; j++) {
                seg->waiting_queue[j - 1] = seg->waiting_queue[j];
            }
            seg->waiter_count--;
            seg->owner_robot_id = next.robot_id;
            seg->state = SEGMENT_RESERVED;
            printf("[RESERVATION] Segment %d promoted to Robot %d\n", msg->segment_id, next.robot_id);
        } else {
            seg->segment_id = -1;
            seg->state = SEGMENT_FREE;
        }
    }
    pthread_mutex_unlock(&reservation_mutex);
}
static void set_fleet_safety_state(int new_state) { // fleet safety state manager
    if (new_state < MANAGER_SAFETY_NORMAL || new_state > MANAGER_SAFETY_EMERGENCY_STOP) {
        printf("[SAFETY] Invalid state requested\n");
        return;
    }
    if (emergency_stop_active && new_state != MANAGER_SAFETY_EMERGENCY_STOP) { // emergency stop cannot be cleared automatically
        printf("[SAFETY] Emergency stop is latched\n");
        return;
    }
    if (fleet_safety_state == new_state) return;
    printf("\n[SAFETY STATE TRANSITION]\n");
    printf("Previous State: %d\n", fleet_safety_state);
    printf("New State: %d\n", new_state);
    fleet_safety_state = new_state;
    switch (fleet_safety_state) {
        case MANAGER_SAFETY_NORMAL:
            printf("[SAFETY] Fleet operating normally\n");
            break;
        case MANAGER_SAFETY_DEGRADED:
            printf("[SAFETY] Restricted operation\n");
            break;
        case MANAGER_SAFETY_SAFE_STOP:
            printf("[SAFETY] SAFE STOP ACTIVATED\n");
            break;
        case MANAGER_SAFETY_RECOVERY:
            printf("[SAFETY] Recovery in progress\n");
            break;
        case MANAGER_SAFETY_EMERGENCY_STOP:
            emergency_stop_active = 1;
            printf("[SAFETY] EMERGENCY STOP LATCHED\n");
            break;
    }
}
static void configure_manager_priority(void) { // manager priority configuration
    struct sched_param param;
    param.sched_priority = MANAGER_PRIORITY;
    int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &param);
    if (rc != 0) {
        printf("[PRIORITY] Configuration failed: %s\n", strerror(rc));
    } else {
        printf("[PRIORITY] Manager priority configured: %d\n", MANAGER_PRIORITY);
    }
}
static int initialize_manager_timer(int chid) { // initialize periodic manager timer
    struct sigevent event;
    struct _itimer timer_spec;
    manager_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (manager_coid == -1) {
        perror("Manager ConnectAttach failed");
        return -1;
    }
    SIGEV_PULSE_INIT(&event, manager_coid, SIGEV_PULSE_PRIO_INHERIT, MANAGER_TIMER_PULSE, 0);
    manager_timer = TimerCreate(CLOCK_MONOTONIC, &event);
    if (manager_timer == -1) {
        perror("Manager TimerCreate failed");
        ConnectDetach(manager_coid);
        manager_coid = -1;
        return -1;
    }
    memset(&timer_spec, 0, sizeof(timer_spec));
    timer_spec.nsec = (long long)MANAGER_TIMER_MS * 1000000LL;
    timer_spec.interval_nsec = (long long)MANAGER_TIMER_MS * 1000000LL;
    if (TimerSettime(manager_timer, 0, &timer_spec, NULL) == -1) {
        perror("Manager TimerSettime failed");
        TimerDestroy(manager_timer);
        ConnectDetach(manager_coid);
        manager_coid = -1;
        return -1;
    }
    timer_initialized = 1;
    printf("[TIMER] Periodic monitoring started: %d ms\n", MANAGER_TIMER_MS);
    return 0;
}
static int validate_message(const robot_message *msg) { // message validation
    if (msg->type != MSG_ROBOT_REPORT) return 0;
    if (msg->robot_id != 1 && msg->robot_id != 2) return 0;
    if (msg->scenario < SCENARIO_JUNCTION || msg->scenario > SCENARIO_CORRIDOR) return 0;
    if (msg->segment_id <= 0) return 0;
    if (msg->direction != 1 && msg->direction != 2) return 0;
    if (msg->speed < 0.0f) return 0;
    if (msg->status < STATUS_MOVING || msg->status > STATUS_STOPPED) return 0;
    return 1;
}
static int validate_freshness(const robot_message *msg, const robot_state *state, uint64_t now) { // message freshness validation
    if (!state->seen) return 1; // first report from this rover
    if (msg->sequence <= state->last_sequence) return 0; // reject duplicate or out-of-order sequence
    if (msg->heartbeat_count <= state->last_heartbeat) return 0; // reject duplicate or decreasing heartbeat
    if (msg->timestamp_ms <= state->last_timestamp_ms) return 0; // reject old or decreasing timestamp
    if (msg->timestamp_ms > now) return 0; // reject future timestamps
    if ((now - msg->timestamp_ms) > MAX_MESSAGE_AGE_MS) return 0; // reject messages that are too old
    return 1;
}
static void update_robot_state(const robot_message *msg, uint64_t receive_time) { // update robot state
    int index = msg->robot_id - 1;
    pthread_mutex_lock(&robot_mutex);
    robots[index].active = 1;
    robots[index].seen = 1;
    robots[index].last_sequence = msg->sequence;
    robots[index].last_heartbeat = msg->heartbeat_count;
    robots[index].last_timestamp_ms = msg->timestamp_ms;
    robots[index].last_receive_ms = receive_time;
    robots[index].status = msg->status;
    pthread_mutex_unlock(&robot_mutex);
}
static void check_heartbeats(void) { // heartbeat monitoring
    uint64_t now = get_time_ms();
    int timeout_detected = 0;
    int i;
    if (now == 0) return;
    pthread_mutex_lock(&robot_mutex);
    for (i = 0; i < ROBOT_COUNT_LOCAL; i++) {
        if (!robots[i].active) continue;
        uint64_t elapsed = now - robots[i].last_receive_ms;
        if (elapsed > HEARTBEAT_TIMEOUT_MS) {
            printf("\n[HEARTBEAT TIMEOUT] Robot %d\n", i + 1);
            printf("No report received for %llu ms\n", (unsigned long long)elapsed);
            robots[i].active = 0;
            robots[i].status = STATUS_STOPPED;
            timeout_detected = 1;
        } else {
            printf("[HEARTBEAT] Robot %d | Sequence: %u | Status: %d\n", i + 1, robots[i].last_sequence, robots[i].status);
        }
    }
    pthread_mutex_unlock(&robot_mutex);
    if (timeout_detected) { // avoid triggering safety state transition while holding robot_mutex
        set_fleet_safety_state(MANAGER_SAFETY_SAFE_STOP);
    }
}
static robot_reply make_initial_decision(const robot_message *msg) { // fleet manager decision engine
    robot_reply reply = {0};
    int result;
    reply.segment_id = msg->segment_id;
    reply.reservation_granted = 0;
    if (fleet_safety_state == MANAGER_SAFETY_EMERGENCY_STOP) { // emergency stop has highest precedence
        reply.command = CMD_STOP;
        reply.reason = REASON_INVALID_REQUEST;
        return reply;
    }
    if (fleet_safety_state == MANAGER_SAFETY_SAFE_STOP || fleet_safety_state == MANAGER_SAFETY_RECOVERY) { // safe stop and recovery prohibit movement
        reply.command = CMD_STOP;
        reply.reason = REASON_HEARTBEAT_TIMEOUT;
        return reply;
    }
    result = request_segment_reservation(msg->robot_id, msg->segment_id, msg->priority); // request shared segment reservation before allowing movement
    if (result == -1) {
        reply.command = CMD_STOP;
        reply.reason = REASON_INVALID_REQUEST;
        return reply;
    }
    if (result == 0) {
        reply.command = CMD_STOP;
        reply.reason = REASON_CONFLICT;
        printf("[MANAGER] Robot %d waiting for Segment %d\n", msg->robot_id, msg->segment_id);
        return reply;
    }
    reply.reservation_granted = 1;
    reply.command = CMD_MOVE;
    reply.reason = REASON_CLEAR;
    if (fleet_safety_state == MANAGER_SAFETY_DEGRADED) { // degraded state permits slow movement only
        reply.command = CMD_SLOW;
        reply.reason = REASON_UNSAFE_DISTANCE;
    }
    printf("[MANAGER] Segment %d granted to Robot %d\n", msg->segment_id, msg->robot_id);
    return reply;
}
static void cleanup_manager(name_attach_t *attach, pid_t robot1_pid, pid_t robot2_pid) { // cleanup function
    if (robot1_pid > 0) kill(robot1_pid, SIGTERM);
    if (robot2_pid > 0) kill(robot2_pid, SIGTERM);
    if (timer_initialized) {
        TimerDestroy(manager_timer);
        timer_initialized = 0;
    }
    if (manager_coid != -1) {
        ConnectDetach(manager_coid);
        manager_coid = -1;
    }
    if (attach != NULL) name_detach(attach, 0);
    if (fw_manager_coid != -1) {
        fw_client_close(fw_manager_coid);
        fw_manager_coid = -1;
    }
}
int main(void) { // main fleet manager
    name_attach_t *attach = NULL;
    rcvid_t rcvid;
    pid_t robot1_pid = -1;
    pid_t robot2_pid = -1;
    printf("\n====================================\n");
    printf("       FLEETSHIELD MANAGER\n");
    printf("====================================\n");
    memset(robots, 0, sizeof(robots));
    initialize_reservation_table();
    fleet_safety_state = MANAGER_SAFETY_NORMAL;
    emergency_stop_active = 0;
    printf("[SAFETY] Initial state: NORMAL\n");
    attach = name_attach(NULL, SERVER_NAME, 0); // register manager server
    if (attach == NULL) {
        perror("name_attach failed");
        return EXIT_FAILURE;
    }
    printf("[MANAGER] Server registered: %s\n", SERVER_NAME);
    printf("[MANAGER] Channel ID: %d\n", attach->chid);
    fw_manager_coid = fw_client_init(); // connect to fleet watchdog
    if (fw_manager_coid == -1) {
        perror("Fleet Watchdog connection failed");
        cleanup_manager(attach, -1, -1);
        return EXIT_FAILURE;
    }
    printf("[FLEET WATCHDOG] Client connected\n");
    configure_manager_priority(); // configure manager priority
    if (initialize_manager_timer(attach->chid) == -1) { // initialize manager timer
        cleanup_manager(attach, -1, -1);
        return EXIT_FAILURE;
    }
    char *robot1_args[] = {"rover", "1", NULL}; // start rover processes
    char *robot2_args[] = {"rover", "2", NULL};
    printf("\n[MANAGER] Starting Robot 1...\n");
    robot1_pid = spawnv(P_NOWAIT, ROBOT_EXECUTABLE, robot1_args);
    if (robot1_pid == -1) {
        perror("Robot 1 spawn failed");
        cleanup_manager(attach, -1, -1);
        return EXIT_FAILURE;
    }
    printf("[MANAGER] Robot 1 started | PID: %ld\n", (long)robot1_pid);
    printf("\n[MANAGER] Starting Robot 2...\n");
    robot2_pid = spawnv(P_NOWAIT, ROBOT_EXECUTABLE, robot2_args);
    if (robot2_pid == -1) {
        perror("Robot 2 spawn failed");
        cleanup_manager(attach, robot1_pid, -1);
        return EXIT_FAILURE;
    }
    printf("[MANAGER] Robot 2 started | PID: %ld\n", (long)robot2_pid);
    printf("\n[MANAGER] Both Rovers launched\n");
    printf("[MANAGER] Waiting for Rover messages...\n");
    while (1) { // main message receiving loop
        union {
            struct _pulse pulse;
            robot_message message;
        } received;
        rcvid = MsgReceive(attach->chid, &received, sizeof(received), NULL);
        if (rcvid == -1) { // message receive error
            if (errno == EINTR) continue;
            perror("MsgReceive failed");
            break;
        }
        if (rcvid == 0) { // handle timer pulse
            if (received.pulse.code == MANAGER_TIMER_PULSE) {
                printf("\n[MANAGER TIMER] Monitoring event\n");
                check_heartbeats(); // monitor rover heartbeats
                if (fw_client_heartbeat(fw_manager_coid, FW_ROLE_MANAGER, ++fw_manager_sequence) != FW_ACK_OK) { // send manager heartbeat to fleet watchdog
                    printf("[MANAGER] Fleet Watchdog heartbeat failed\n");
                    set_fleet_safety_state(MANAGER_SAFETY_SAFE_STOP);
                }
            }
            continue;
        }
        robot_message msg = received.message; // copy received message
        if (!validate_message(&msg)) { // validate message format
            printf("[MANAGER] Invalid message received\n");
            MsgError(rcvid, EINVAL);
            continue;
        }
        uint64_t now = get_time_ms(); // validate message freshness
        if (now == 0) {
            MsgError(rcvid, EIO);
            continue;
        }
        int index = msg.robot_id - 1;
        pthread_mutex_lock(&robot_mutex);
        int fresh = validate_freshness(&msg, &robots[index], now);
        pthread_mutex_unlock(&robot_mutex);
        if (!fresh) {
            printf("\n[STALE MESSAGE] Robot %d\n", msg.robot_id);
            robot_reply stale_reply = {0};
            stale_reply.command = CMD_STOP;
            stale_reply.reason = REASON_STALE_MESSAGE;
            stale_reply.reservation_granted = 0;
            stale_reply.segment_id = msg.segment_id;
            if (MsgReply(rcvid, EOK, &stale_reply, sizeof(stale_reply)) == -1) {
                perror("MsgReply for stale message failed");
            }
            continue;
        }
        update_robot_state(&msg, get_time_ms()); // update robot state
        process_segment_status(&msg); // process segment status
        printf("\n====================================\n"); // display robot information
        printf("ROBOT %d REPORT RECEIVED\n", msg.robot_id);
        printf("Position: X = %.2f, Y = %.2f\n", msg.current_x, msg.current_y);
        printf("Destination: X = %.2f, Y = %.2f\n", msg.destination_x, msg.destination_y);
        printf("Speed: %.2f\n", msg.speed);
        printf("Priority: %d\n", msg.priority);
        printf("Status: %d\n", msg.status);
        printf("Scenario: %d\n", msg.scenario);
        printf("Segment ID: %d\n", msg.segment_id);
        printf("Direction: %d\n", msg.direction);
        printf("Sequence: %u\n", msg.sequence);
        printf("Heartbeat: %u\n", msg.heartbeat_count);
        printf("Timestamp: %llu\n", (unsigned long long)msg.timestamp_ms);
        robot_reply reply = make_initial_decision(&msg); // make fleet manager decision
        if (MsgReply(rcvid, EOK, &reply, sizeof(reply)) == -1) { // send reply to rover
            perror("MsgReply failed");
            break;
        }
        printf("[MANAGER] Reply sent to Robot %d | Command: %d\n", msg.robot_id, reply.command);
    }
    printf("\n[MANAGER] Shutting down...\n"); // cleanup
    cleanup_manager(attach, robot1_pid, robot2_pid);
    pthread_mutex_destroy(&robot_mutex);
    pthread_mutex_destroy(&reservation_mutex);
    return EXIT_SUCCESS;
}
