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
#include "fleet_common.h"
static uint64_t get_time_ms(void);
#define MANAGER_TIMER_PULSE 10 // manager configuration
#define MANAGER_TIMER_MS 1000
#define MAX_MESSAGE_AGE_MS 3000ULL
#define ROBOT1_INDEX 0
#define ROBOT2_INDEX 1
#define MANAGER_PRIORITY 20
#define ROBOT_EXECUTABLE "/tmp/robot_bin/rover"
static timer_t manager_timer; // global timer and ipc variables
static int manager_coid = -1;
static int timer_initialized = 0;
static pthread_mutex_t robot_mutex = PTHREAD_MUTEX_INITIALIZER;
static int fleet_safety_state = SAFETY_NORMAL; // central safety manager
static int emergency_stop_active = 0;
typedef struct { // robot state info
    int active;
    int seen;
    uint32_t last_sequence;
    uint32_t last_heartbeat;
    int status;
    uint64_t last_timestamp_ms;
    uint64_t last_receive_ms;
} robot_state;
static robot_state robots[2] = {0};
#define MAX_SEGMENTS 10 // shared segment reservation table
typedef struct { // priority-based waiting request
    int robot_id;
    int priority;
    uint64_t request_timestamp_ms;
} waiting_request;
#define MAX_WAITING_ROBOTS ROBOT_COUNT // shared segment reservation
typedef struct {
    int segment_id;
    int state;
    int owner_robot_id;
    int waiter_count;
    waiting_request waiting_queue[MAX_WAITING_ROBOTS];
} segment_reservation;
static segment_reservation reservation_table[MAX_SEGMENTS];
static pthread_mutex_t reservation_mutex = PTHREAD_MUTEX_INITIALIZER;
static void initialize_reservation_table(void) { // initialize reservation table
    for (int i = 0; i < MAX_SEGMENTS; i++) {
        reservation_table[i].segment_id = -1;
        reservation_table[i].state = SEGMENT_FREE;
        reservation_table[i].owner_robot_id = 0;
        reservation_table[i].waiter_count = 0;
        for (int j = 0; j < MAX_WAITING_ROBOTS; j++) {
            reservation_table[i].waiting_queue[j].robot_id = 0;
            reservation_table[i].waiting_queue[j].priority = 0;
            reservation_table[i].waiting_queue[j].request_timestamp_ms = 0;
        }
    }
    printf("[RESERVATION] Priority queue initialized\n");
}
static int request_segment_reservation(int robot_id, int segment_id, int priority) { // priority-based reservation request
    uint64_t request_time = get_time_ms();
    if ((robot_id != 1 && robot_id != 2) || segment_id <= 0 || priority < 0 || request_time == 0) {
        return -1;
    }
    pthread_mutex_lock(&reservation_mutex);
    int slot = -1;
    int free_slot = -1;
    for (int i = 0; i < MAX_SEGMENTS; i++) { // find existing segment or unused slot
        if (reservation_table[i].segment_id == segment_id) {
            slot = i;
            break;
        }
        if (reservation_table[i].state == SEGMENT_FREE && reservation_table[i].owner_robot_id == 0 && reservation_table[i].waiter_count == 0 && free_slot == -1) {
            free_slot = i;
        }
    }
    if (slot == -1) { // create a new segment entry
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
    if (segment->owner_robot_id == robot_id) { // existing owner retains ownership
        pthread_mutex_unlock(&reservation_mutex);
        return 1;
    }
    for (int i = 0; i < segment->waiter_count; i++) { // already waiting, preserve original timestamp
        if (segment->waiting_queue[i].robot_id == robot_id) {
            pthread_mutex_unlock(&reservation_mutex);
            return 0;
        }
    }
    if (segment->state == SEGMENT_FREE && segment->owner_robot_id == 0 && segment->waiter_count == 0) { // segment free, grant immediately if no queue
        segment->owner_robot_id = robot_id;
        segment->state = SEGMENT_RESERVED;
        printf("[RESERVATION] Segment %d granted to Robot %d\n", segment_id, robot_id);
        pthread_mutex_unlock(&reservation_mutex);
        return 1;
    }
    if (segment->waiter_count >= MAX_WAITING_ROBOTS) { // add request to waiting queue
        pthread_mutex_unlock(&reservation_mutex);
        return -1;
    }
    int pos = segment->waiter_count;
    segment->waiting_queue[pos].robot_id = robot_id;
    segment->waiting_queue[pos].priority = priority;
    segment->waiting_queue[pos].request_timestamp_ms = request_time;
    segment->waiter_count++;
    for (int i = segment->waiter_count - 1; i > 0; i--) { // sort higher priority first, earlier timestamp first
        waiting_request *current = &segment->waiting_queue[i];
        waiting_request *previous = &segment->waiting_queue[i - 1];
        int should_swap = (current->priority > previous->priority) || (current->priority == previous->priority && current->request_timestamp_ms < previous->request_timestamp_ms);
        if (!should_swap) break;
        waiting_request temp = *current;
        *current = *previous;
        *previous = temp;
    }
    printf("[RESERVATION] Robot %d queued for Segment %d | Priority: %d\n", robot_id, segment_id, priority);
    printf("[RESERVATION] Queue position: %d\n", pos + 1);
    pthread_mutex_unlock(&reservation_mutex);
    return 0;
}
static uint64_t get_time_ms(void) { // monotonic clock in ms
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1) {
        perror("clock_gettime");
        return 0;
    }
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}
static void set_fleet_safety_state(int new_state) { // central safety state manager
    if (new_state < SAFETY_NORMAL || new_state > SAFETY_EMERGENCY_STOP) {
        printf("[SAFETY] Invalid state requested\n");
        return;
    }
    if (emergency_stop_active && new_state != SAFETY_EMERGENCY_STOP) { // emergency stop cannot be cleared automatically
        printf("[SAFETY] Emergency stop is latched\n");
        return;
    }
    if (fleet_safety_state == new_state) return;
    printf("\n[SAFETY STATE TRANSITION]\n");
    printf("Previous State: %d\n", fleet_safety_state);
    printf("New State: %d\n", new_state);
    fleet_safety_state = new_state;
    switch (fleet_safety_state) {
        case SAFETY_NORMAL:
            printf("[SAFETY] Fleet operating normally\n");
            break;
        case SAFETY_DEGRADED:
            printf("[SAFETY] Restricted operation\n");
            break;
        case SAFETY_SAFE_STOP:
            printf("[SAFETY] SAFE STOP ACTIVATED\n");
            break;
        case SAFETY_RECOVERY:
            printf("[SAFETY] Recovery in progress\n");
            break;
        case SAFETY_EMERGENCY_STOP:
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
    timer_spec.nsec = 1000000000ULL;
    timer_spec.interval_nsec = 1000000000ULL;
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
    if (msg->sequence <= state->last_sequence) return 0; // reject duplicate or out of order sequence
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
    if (now == 0) return;
    pthread_mutex_lock(&robot_mutex);
    for (int i = 0; i < 2; i++) {
        if (!robots[i].active) continue;
        uint64_t elapsed = now - robots[i].last_receive_ms;
        if (elapsed > HEARTBEAT_TIMEOUT_MS) {
            printf("\n[HEARTBEAT TIMEOUT] Robot %d\n", i + 1);
            printf("No report received for %llu ms\n", (unsigned long long)elapsed);
            robots[i].active = 0;
            robots[i].status = STATUS_STOPPED;
            set_fleet_safety_state(SAFETY_SAFE_STOP);
        } else {
            printf("[HEARTBEAT] Robot %d | Sequence: %u | Status: %d\n", i + 1, robots[i].last_sequence, robots[i].status);
        }
    }
    pthread_mutex_unlock(&robot_mutex);
}
static robot_reply make_initial_decision(const robot_message *msg) { // safety-aware reservation decision
    robot_reply reply = {0};
    reply.segment_id = msg->segment_id;
    reply.reservation_granted = 0;
    if (fleet_safety_state == SAFETY_EMERGENCY_STOP) { // emergency stop has highest precedence
        reply.command = CMD_STOP;
        reply.reason = REASON_INVALID_REQUEST;
        return reply;
    }
    if (fleet_safety_state == SAFETY_SAFE_STOP || fleet_safety_state == SAFETY_RECOVERY) { // fleet safe stop
        reply.command = CMD_STOP;
        reply.reason = REASON_HEARTBEAT_TIMEOUT;
        return reply;
    }
    int result = request_segment_reservation(msg->robot_id, msg->segment_id, msg->priority); // request segment reservation and update occupancy
    if (result == -1) { // invalid request or queue full
        reply.command = CMD_STOP;
        reply.reason = REASON_INVALID_REQUEST;
        return reply;
    }
    if (result == 0) { // reservation not yet available
        reply.command = CMD_STOP;
        reply.reason = REASON_CONFLICT;
        printf("[MANAGER] Robot %d waiting for Segment %d\n", msg->robot_id, msg->segment_id);
        return reply;
    }
    reply.reservation_granted = 1; // reservation granted
    reply.command = CMD_MOVE;
    reply.reason = REASON_CLEAR;
    if (fleet_safety_state == SAFETY_DEGRADED) { // degraded mode permits reservation but moves slowly
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
    pthread_mutex_destroy(&robot_mutex);
}
int main(void) { // main fleet manager
    name_attach_t *attach = NULL;
    rcvid_t rcvid;
    pid_t robot1_pid = -1;
    pid_t robot2_pid = -1;
    printf("\n====================================\n");
    printf("     FLEETSHIELD MANAGER STARTED\n");
    printf("====================================\n");
    memset(robots, 0, sizeof(robots));
    initialize_reservation_table();
    fleet_safety_state = SAFETY_NORMAL;
    emergency_stop_active = 0;
    printf("[SAFETY] Initial state: NORMAL\n");
    attach = name_attach(NULL, SERVER_NAME, 0); // register manager server
    if (attach == NULL) {
        perror("name_attach failed");
        return EXIT_FAILURE;
    }
    printf("[MANAGER] Server registered: %s\n", SERVER_NAME);
    printf("[MANAGER] Channel ID: %d\n", attach->chid);
    configure_manager_priority(); // configure manager priority
    if (initialize_manager_timer(attach->chid) == -1) { // initialize periodic timer
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
    printf("\n[MANAGER] Both Rovers launched successfully!\n");
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
                check_heartbeats();
            }
            continue;
        }
        robot_message msg = received.message; // copy received message without using a global variable
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
        printf("\n====================================\n"); // display robot info
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
        robot_reply reply = make_initial_decision(&msg); // make manager decision
        if (MsgReply(rcvid, EOK, &reply, sizeof(reply)) == -1) { // send reply to rover
            perror("MsgReply failed");
            break;
        }
        printf("[MANAGER] Reply sent to Robot %d | Command: %d\n", msg.robot_id, reply.command);
    }
    printf("\n[MANAGER] Shutting down...\n"); // cleanup
    cleanup_manager(attach, robot1_pid, robot2_pid);
    return EXIT_SUCCESS;
}
