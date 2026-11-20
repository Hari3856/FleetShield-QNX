#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fleet_common.h"
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
typedef struct { // task management structures
    int robot_id;
    int heartbeat;
    float source_x;
    float source_y;
    float dest_x;
    float dest_y;
    float curr_x;
    float curr_y;
    int robot_seq;
    int robot_status;
    float speed;
    int direction;
} robot;
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float front_dist;
    float left_dist;
    float right_dist;
    int obstacle_detected;
} sensor;
typedef enum {
    TASK_WAITING,
    TASK_INPROGRESS,
    TASK_COMPLETE,
    TASK_FAILED
} taskstatus;
typedef struct {
    char taskid[10];
    int priority;
    taskstatus status;
    unsigned long long timestamp;
} task;
typedef struct {
    robot robot_data;
    task task_data;
    sensor sensor_Data;
} report;
static int task_priority(const char *id) { // task priority management
    static const char *const ids[] = {"t1", "t2", "t3", "t4", "t5"};
    static const int priorities[] = {10, 20, 30, 40, 50};
    if (id == NULL) return -1;
    for (int i = 0; i < 5; i++) {
        if (strcmp(id, ids[i]) == 0) return priorities[i];
    }
    return -1;
}
static void initialise_task(task tasks[5]) {
    const char *ids[5] = {"t1", "t2", "t3", "t4", "t5"};
    for (int i = 0; i < 5; i++) {
        strcpy(tasks[i].taskid, ids[i]);
        tasks[i].priority = task_priority(ids[i]);
        tasks[i].status = TASK_WAITING;
        tasks[i].timestamp = 0;
    }
}
static int get_timestamp_ms(uint64_t *timestamp) { // get monotonic timestamp in ms
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1) {
        perror("clock_gettime");
        return -1;
    }
    *timestamp = ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
    return 0;
}
static void update_position(int id, float *current_x, float *current_y, float current_speed, float destination_x, float destination_y) { // update simulated robot position
    if (id == 1) {
        *current_x += current_speed;
        if (*current_x > destination_x) *current_x = destination_x;
    } else {
        *current_y += current_speed;
        if (*current_y > destination_y) *current_y = destination_y;
    }
}
int main(int argc, char *argv[]) { // main rover process
    int coid;
    int id;
    float source_x, source_y;
    float destination_x, destination_y;
    float current_x, current_y;
    float normal_speed = 0.5f;
    float current_speed = 0.5f;
    int current_status = STATUS_MOVING;
    int priority;
    uint32_t sequence = 0;
    uint32_t heartbeat_count = 0;
    task robot_tasks[5];
    if (argc != 2) { // validate command-line args
        printf("Usage: rover <robot_id>\n");
        return EXIT_FAILURE;
    }
    id = atoi(argv[1]);
    if (id != 1 && id != 2) {
        printf("Invalid Robot ID. Use 1 or 2.\n");
        return EXIT_FAILURE;
    }
    initialise_task(robot_tasks); // init tasks
    printf("\n========== FLEETSHIELD ROVER ==========\n");
    printf("Robot ID: %d\n", id);
    printf("Process ID: %ld\n", (long)getpid());
    coid = name_open(SERVER_NAME, 0); // connect to fleet manager
    if (coid == -1) {
        perror("name_open failed");
        return EXIT_FAILURE;
    }
    printf("Robot %d connected to FleetShield Manager\n", id);
    if (id == 1) { // init robot paths
        source_x = 1.0f;
        source_y = 2.0f;
        destination_x = 5.0f;
        destination_y = 2.0f;
        priority = 2;
    } else {
        source_x = 3.0f;
        source_y = 0.0f;
        destination_x = 3.0f;
        destination_y = 5.0f;
        priority = 1;
    }
    current_x = source_x;
    current_y = source_y;
    while (sequence < 10) { // main communication and movement loop
        robot_message msg = {0};
        robot_reply reply = {0};
        msg.type = MSG_ROBOT_REPORT; // message identification
        msg.robot_id = id;
        msg.source_x = source_x; // robot path info
        msg.source_y = source_y;
        msg.destination_x = destination_x;
        msg.destination_y = destination_y;
        msg.current_x = current_x;
        msg.current_y = current_y;
        msg.speed = current_speed; // movement info
        msg.priority = priority;
        msg.status = current_status;
        msg.scenario = SCENARIO_JUNCTION; // temp junction test config
        msg.segment_id = 101;
        msg.direction = (id == 1) ? 1 : 2;
        msg.segment_occupied = 0;
        msg.segment_clear = 0;
        msg.sensor_distance = 100.0f; // temp sensor values, replace when hardware added
        msg.acceleration_magnitude = 0.0f;
        if (get_timestamp_ms(&msg.timestamp_ms) == -1) { // timestamp and freshness info
            current_status = STATUS_STOPPED;
            current_speed = 0.0f;
            break;
        }
        msg.sequence = sequence;
        msg.heartbeat_count = ++heartbeat_count;
        printf("\nSending report from Robot %d\n", id); // send message to fleet manager (only one msgsend per report)
        int rc = MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply));
        if (rc == -1) { // communication failure
            perror("MsgSend failed");
            current_status = STATUS_STOPPED;
            current_speed = 0.0f;
            break;
        }
        printf("\n========== ROBOT %d REPORT ==========\n", id); // display message info
        printf("Source: (%.2f, %.2f)\n", msg.source_x, msg.source_y);
        printf("Current: (%.2f, %.2f)\n", msg.current_x, msg.current_y);
        printf("Destination: (%.2f, %.2f)\n", msg.destination_x, msg.destination_y);
        printf("Speed: %.2f\n", msg.speed);
        printf("Priority: %d\n", msg.priority);
        printf("Scenario: %d\n", msg.scenario);
        printf("Segment ID: %d\n", msg.segment_id);
        printf("Sequence: %u\n", msg.sequence);
        printf("Heartbeat: %u\n", msg.heartbeat_count);
        printf("Timestamp: %llu\n", (unsigned long long)msg.timestamp_ms);
        printf("Manager command: %d\n", reply.command);
        if (reply.command == CMD_MOVE) { // process manager command
            printf("Manager: MOVE\n");
            current_status = STATUS_MOVING;
            current_speed = normal_speed;
            update_position(id, &current_x, &current_y, current_speed, destination_x, destination_y);
        } else if (reply.command == CMD_SLOW) {
            printf("Manager: SLOW\n");
            current_status = STATUS_SLOWING;
            current_speed = 0.2f;
            update_position(id, &current_x, &current_y, current_speed, destination_x, destination_y);
        } else if (reply.command == CMD_STOP) {
            printf("Manager: STOP - Robot stopped\n");
            current_status = STATUS_STOPPED;
            current_speed = 0.0f; // dont terminate rover, keep reporting heartbeats
        } else if (reply.command == CMD_RESUME) {
            printf("Manager: RESUME\n");
            current_status = STATUS_MOVING;
            current_speed = normal_speed;
            update_position(id, &current_x, &current_y, current_speed, destination_x, destination_y);
        } else {
            printf("Unknown Manager command: %d\n", reply.command);
            current_status = STATUS_STOPPED;
            current_speed = 0.0f;
            break;
        }
        sequence++; // increment once per report even when stopped
        sleep(1); // periodic report interval
    }
    printf("\nRobot %d terminating.\n", id); // cleanup
    name_close(coid);
    return EXIT_SUCCESS;
}
