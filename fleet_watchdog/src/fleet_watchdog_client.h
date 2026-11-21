#ifndef FLEET_WATCHDOG_CLIENT_H
#define FLEET_WATCHDOG_CLIENT_H

#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/dispatch.h>

#define FW_SERVER_NAME "fleet_watchdog"

#define FW_ROLE_MANAGER 1
#define FW_ROLE_ROVER_1 2
#define FW_ROLE_ROVER_2 3

#define FW_MSG_HEARTBEAT 1
#define FW_ACK_OK 0

typedef struct {
    int message_type;
    int role;
    int process_id;
    uint32_t sequence;
    uint64_t timestamp_ms;
} fw_heartbeat_message;

typedef struct {
    int status;
} fw_heartbeat_reply;

static inline int fw_client_init(void)
{
    return name_open(FW_SERVER_NAME, 0);
}

static inline int fw_client_heartbeat(
    int coid,
    int role,
    uint32_t sequence)
{
    fw_heartbeat_message msg;
    fw_heartbeat_reply reply;

    struct timespec ts;

    if (coid == -1)
        return -1;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    msg.message_type = FW_MSG_HEARTBEAT;
    msg.role = role;
    msg.process_id = getpid();
    msg.sequence = sequence;
    msg.timestamp_ms =
        ((uint64_t)ts.tv_sec * 1000ULL) +
        ((uint64_t)ts.tv_nsec / 1000000ULL);

    int status = MsgSend(
        coid,
        &msg,
        sizeof(msg),
        &reply,
        sizeof(reply)
    );

    if (status == -1)
        return -1;

    return reply.status;
}

static inline void fw_client_close(int coid)
{
    if (coid != -1)
        name_close(coid);
}

#endif
