#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <process.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#define RECOVERY_CHANNEL "fleet_recovery" // fleetshield recovery manager supervising rovers and fleet manager
#define PROCESS_COUNT 3
#define HEARTBEAT_TIMEOUT_MS 3000ULL
#define MONITOR_INTERVAL_MS 500
#define STARTUP_TIMEOUT_MS 5000ULL
#define MAX_RESTART_ATTEMPTS 3
#define MSG_HEARTBEAT 1
#define ROLE_ROVER_1 1
#define ROLE_ROVER_2 2
#define ROLE_MANAGER 3
#define STATE_STARTING 0
#define STATE_HEALTHY 1
#define STATE_FAILED 2
#define STATE_RECOVERING 3
#define STATE_SAFE_WAIT 4
#define ACK_OK 0
#define ACK_INVALID -1
typedef struct {
    int message_type;
    int role;
    uint32_t sequence;
} heartbeat_message;
typedef struct {
    int role;
    const char *name;
    const char *path;
    char *const *arguments;
    pid_t pid;
    int state;
    int restart_attempts;
    uint32_t last_sequence;
    uint64_t last_heartbeat_ms;
    uint64_t restart_time_ms;
    int heartbeat_seen;
} process_info;
static name_attach_t *attach = NULL;
static process_info processes[PROCESS_COUNT];
static uint64_t get_time_ms(void) { // monotonic time in milliseconds
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1) {
        perror("clock_gettime");
        return 0;
    }
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}
static char *rover1_args[] = {"rover", "1", NULL}; // process configuration (update paths per qnx deployment directory)
static char *rover2_args[] = {"rover", "2", NULL};
static char *manager_args[] = {"fleet_manager", NULL};
static void initialize_process_table(void) {
    memset(processes, 0, sizeof(processes));
    processes[0].role = ROLE_ROVER_1;
    processes[0].name = "ROVER 1";
    processes[0].path = "/tmp/robot_bin/rover";
    processes[0].arguments = rover1_args;
    processes[1].role = ROLE_ROVER_2;
    processes[1].name = "ROVER 2";
    processes[1].path = "/tmp/robot_bin/rover";
    processes[1].arguments = rover2_args;
    processes[2].role = ROLE_MANAGER;
    processes[2].name = "FLEET MANAGER";
    processes[2].path = "/tmp/robot_bin/fleet_manager";
    processes[2].arguments = manager_args;
    for (int i = 0; i < PROCESS_COUNT; i++) {
        processes[i].pid = -1;
        processes[i].state = STATE_STARTING;
        processes[i].restart_attempts = 0;
        processes[i].last_sequence = 0;
        processes[i].last_heartbeat_ms = 0;
        processes[i].restart_time_ms = 0;
        processes[i].heartbeat_seen = 0;
    }
}
static int find_process(int role) { // find process using role
    for (int i = 0; i < PROCESS_COUNT; i++) {
        if (processes[i].role == role) return i;
    }
    return -1;
}
static int start_process(int index) { // start or restart process
    process_info *p = &processes[index];
    printf("\n[RECOVERY] Starting %s\n", p->name);
    pid_t new_pid = spawnv(P_NOWAIT, p->path, p->arguments);
    if (new_pid == -1) {
        perror("[RECOVERY] spawnv failed");
        p->state = STATE_FAILED;
        return -1;
    }
    p->pid = new_pid;
    p->state = STATE_STARTING;
    p->heartbeat_seen = 0;
    p->last_sequence = 0;
    p->last_heartbeat_ms = 0;
    p->restart_time_ms = get_time_ms();
    printf("[RECOVERY] %s started | PID = %ld\n", p->name, (long)new_pid);
    return 0;
}
static void terminate_process(process_info *p) { // terminate process
    if (p->pid <= 0) return;
    printf("[RECOVERY] Terminating %s | PID = %ld\n", p->name, (long)p->pid);
    if (kill(p->pid, SIGTERM) == -1 && errno != ESRCH) {
        perror("[RECOVERY] SIGTERM failed");
    }
    for (int i = 0; i < 10; i++) { // bounded wait for process termination to prevent indefinite blocking
        int status;
        pid_t result = waitpid(p->pid, &status, WNOHANG);
        if (result == p->pid || (result == -1 && errno == ECHILD)) break;
        if (result == -1 && errno != EINTR) break;
        delay(100);
    }
    if (kill(p->pid, 0) == 0) { // escalate if the process did not exit
        if (kill(p->pid, SIGKILL) == -1 && errno != ESRCH) perror("[RECOVERY] SIGKILL failed");
        waitpid(p->pid, NULL, 0);
    }
    p->pid = -1;
}
static void recover_process(int index) { // handle process failure
    process_info *p = &processes[index];
    if (p->state == STATE_RECOVERING) return;
    printf("\n[FAULT DETECTED] %s\n", p->name);
    p->state = STATE_RECOVERING;
    terminate_process(p);
    if (p->restart_attempts >= MAX_RESTART_ATTEMPTS) {
        printf("[RECOVERY FAILED] %s exceeded restart limit\n", p->name);
        p->state = STATE_FAILED;
        return;
    }
    p->restart_attempts++;
    printf("[RECOVERY] Attempt %d of %d for %s\n", p->restart_attempts, MAX_RESTART_ATTEMPTS, p->name);
    if (start_process(index) == -1) {
        printf("[RECOVERY FAILED] Could not restart %s\n", p->name);
        p->state = STATE_FAILED;
    }
}
static void process_heartbeat(const heartbeat_message *msg) { // process heartbeat handler
    int index = find_process(msg->role);
    if (index == -1) {
        printf("[HEARTBEAT] Invalid process role\n");
        return;
    }
    process_info *p = &processes[index];
    if (msg->sequence == 0) return;
    if (p->heartbeat_seen && msg->sequence <= p->last_sequence) {
        printf("[HEARTBEAT] Invalid sequence from %s\n", p->name);
        return;
    }
    p->last_sequence = msg->sequence;
    p->last_heartbeat_ms = get_time_ms();
    p->heartbeat_seen = 1;
    if (p->state == STATE_STARTING || p->state == STATE_RECOVERING) {
        printf("\n[RECOVERY VERIFIED] %s | PID = %ld\n", p->name, (long)p->pid);
        p->state = STATE_HEALTHY; // keep restart_attempts intact until a stable healthy period passes
    } else {
        p->state = STATE_HEALTHY;
    }
    printf("[HEARTBEAT] %s | Sequence = %u\n", p->name, p->last_sequence);
}
static void check_heartbeat_timeouts(void) { // check heartbeat timeouts
    uint64_t now = get_time_ms();
    if (now == 0) return;
    for (int i = 0; i < PROCESS_COUNT; i++) {
        process_info *p = &processes[i];
        if (p->state == STATE_FAILED || p->state == STATE_RECOVERING) continue;
        if (p->state == STATE_STARTING) {
            if (now - p->restart_time_ms > STARTUP_TIMEOUT_MS) {
                printf("\n[STARTUP TIMEOUT] %s\n", p->name);
                recover_process(i);
            }
            continue;
        }
        if (!p->heartbeat_seen) continue;
        uint64_t elapsed = now - p->last_heartbeat_ms;
        if (elapsed > HEARTBEAT_TIMEOUT_MS) {
            printf("\n[HEARTBEAT TIMEOUT] %s | %llu ms\n", p->name, (unsigned long long)elapsed);
            recover_process(i);
        }
    }
}
static void check_process_exits(void) { // check whether process exited
    for (int i = 0; i < PROCESS_COUNT; i++) {
        process_info *p = &processes[i];
        if (p->pid <= 0 || p->state == STATE_FAILED || p->state == STATE_RECOVERING) continue;
        int status;
        pid_t result = waitpid(p->pid, &status, WNOHANG);
        if (result == p->pid) {
            printf("\n[PROCESS EXIT] %s | PID = %ld\n", p->name, (long)p->pid);
            p->pid = -1;
            recover_process(i);
        } else if (result == -1 && errno == ECHILD) {
            p->pid = -1; // child is no longer waitable
            recover_process(i);
        }
    }
}
static void print_status(void) { // print process status
    printf("\n========== FLEET RECOVERY STATUS ==========\n");
    for (int i = 0; i < PROCESS_COUNT; i++) {
        process_info *p = &processes[i];
        const char *state_name = "UNKNOWN";
        switch (p->state) {
            case STATE_STARTING:
                state_name = "STARTING";
                break;
            case STATE_HEALTHY:
                state_name = "HEALTHY";
                break;
            case STATE_FAILED:
                state_name = "FAILED";
                break;
            case STATE_RECOVERING:
                state_name = "RECOVERING";
                break;
            case STATE_SAFE_WAIT:
                state_name = "SAFE_WAIT";
                break;
        }
        printf("%-18s | PID: %-6ld | State: %-12s | Restarts: %d\n", p->name, (long)p->pid, state_name, p->restart_attempts);
    }
}
static void cleanup(void) { // cleanup
    printf("\n[RECOVERY] Shutting down supervisor\n");
    for (int i = 0; i < PROCESS_COUNT; i++) {
        terminate_process(&processes[i]);
    }
    if (attach != NULL) {
        name_detach(attach, 0);
        attach = NULL;
    }
}
int main(void) { // main recovery manager
    printf("\n========================================\n");
    printf("     FLEETSHIELD RECOVERY MANAGER\n");
    printf("========================================\n");
    initialize_process_table();
    attach = name_attach(NULL, RECOVERY_CHANNEL, 0); // register named qnx ipc channel
    if (attach == NULL) {
        perror("name_attach failed");
        return EXIT_FAILURE;
    }
    printf("[RECOVERY] IPC channel registered: %s\n", RECOVERY_CHANNEL);
    for (int i = 0; i < PROCESS_COUNT; i++) { // start all supervised processes
        if (start_process(i) == -1) {
            printf("[RECOVERY] Initial startup failed for %s\n", processes[i].name);
            cleanup();
            return EXIT_FAILURE;
        }
    }
    printf("\n[RECOVERY] Monitoring all processes...\n");
    while (1) {
        union {
            struct _pulse pulse;
            heartbeat_message heartbeat;
        } received;
        int rcvid = MsgReceive(attach->chid, &received, sizeof(received), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            perror("MsgReceive failed");
            break;
        }
        if (rcvid > 0) {
            if (received.heartbeat.message_type == MSG_HEARTBEAT) {
                process_heartbeat(&received.heartbeat);
                MsgReply(rcvid, ACK_OK, NULL, 0);
            } else {
                MsgError(rcvid, EINVAL);
            }
        }
        check_process_exits();
        check_heartbeat_timeouts();
        print_status();
    }
    cleanup();
    return EXIT_SUCCESS;
}
