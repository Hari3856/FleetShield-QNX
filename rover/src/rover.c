#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <pthread.h> // creates and manages thread
#include <sched.h> // gives scheduling policies and parameters
#include "fleet_common.h"
#include "rover_watchdog.h"
#include "fleet_watchdog_client.h"
typedef struct { // robot and sensor structures
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
typedef enum {
    TASK_READY,
    TASK_RUNNING,
    TASK_PAUSED,
    TASK_FINISHED
} scheduler_state;
typedef struct {
    char taskid[10];
    const char *task_name;
    int priority;
    taskstatus status;
    unsigned long long timestamp;
    unsigned int budget_ms; // cpu budget management
    unsigned int remaining_budget_ms;
    unsigned int remaining_work_ms;
    uint64_t replenish_at_ns;
    scheduler_state state; // scheduler state
} task;
#define TASK_COUNT 5 // qnx sporadic scheduling configuration
#define WORKER_HIGH_PRIORITY 30 // qnx thread scheduling parameters
#define WORKER_LOW_PRIORITY 10
#define WORKER_BUDGET_MS 20
#define WORKER_PERIOD_MS 200
#define WORKER_MAX_REPL 1
static pthread_t scheduler_thread; // worker thread control
static pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;
static int scheduler_running = 1;
#define T1_BUDGET_MS 10 // individual warehouse task budgets
#define T2_BUDGET_MS 15
#define T3_BUDGET_MS 20
#define T4_BUDGET_MS 25
#define T5_BUDGET_MS 10
int select_next_warehouse_task(task tasks[TASK_COUNT]) { // select highest-priority ready task
    int selected = -1;
    for (int i = 0; i < TASK_COUNT; i++) {
        if (tasks[i].state == TASK_READY) {
            if (selected == -1 || tasks[i].priority > tasks[selected].priority) {
                selected = i;
            }
        }
    }
    return selected;
}
static uint64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}
void *scheduler_worker(void *arg) {
    task *tasks = (task *)arg;
    while (scheduler_running) {
        pthread_mutex_lock(&task_mutex);
        uint64_t now = get_time_ns(); // replenish paused tasks when their period expires
        for (int i = 0; i < TASK_COUNT; i++) {
            if (tasks[i].state == TASK_PAUSED && now >= tasks[i].replenish_at_ns) {
                tasks[i].remaining_budget_ms = tasks[i].budget_ms;
                tasks[i].state = TASK_READY;
                printf("[SCHEDULER] Budget replenished: %s\n", tasks[i].task_name);
            }
        }
        int index = select_next_warehouse_task(tasks);
        if (index < 0) {
            pthread_mutex_unlock(&task_mutex);
            usleep(5000);
            continue;
        }
        tasks[index].state = TASK_RUNNING;
        unsigned int slice = 5;
        if (tasks[index].remaining_budget_ms < slice) slice = tasks[index].remaining_budget_ms;
        if (tasks[index].remaining_work_ms < slice) slice = tasks[index].remaining_work_ms;
        printf("\n[SCHEDULER] Running: %s | Priority: %d\n", tasks[index].task_name, tasks[index].priority);
        pthread_mutex_unlock(&task_mutex);
        usleep(slice * 1000); // simulated execution interval
        pthread_mutex_lock(&task_mutex);
        tasks[index].remaining_budget_ms -= slice;
        tasks[index].remaining_work_ms -= slice;
        if (tasks[index].remaining_work_ms == 0) {
            tasks[index].state = TASK_FINISHED;
            tasks[index].status = TASK_COMPLETE;
            printf("[SCHEDULER] %s COMPLETED\n", tasks[index].task_name);
        } else if (tasks[index].remaining_budget_ms == 0) {
            tasks[index].state = TASK_PAUSED;
            tasks[index].replenish_at_ns = get_time_ns() + (uint64_t)WORKER_PERIOD_MS * 1000000ULL;
            printf("[SCHEDULER] %s PAUSED - budget exhausted\n", tasks[index].task_name);
        } else {
            tasks[index].state = TASK_READY;
        }
        pthread_mutex_unlock(&task_mutex);
    }
    return NULL;
}
int start_scheduler(task tasks[TASK_COUNT]) {
    pthread_attr_t attr;
    struct sched_param param = {0};
    struct timespec budget = {0, WORKER_BUDGET_MS * 1000000L};
    struct timespec period = {0, WORKER_PERIOD_MS * 1000000L};
    int ret;
    ret = pthread_attr_init(&attr);
    if (ret != 0) return ret;
    param.sched_priority = WORKER_HIGH_PRIORITY;
    param.sched_ss_low_priority = WORKER_LOW_PRIORITY;
    param.sched_ss_init_budget = budget;
    param.sched_ss_repl_period = period;
    param.sched_ss_max_repl = WORKER_MAX_REPL;
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret == 0) ret = pthread_attr_setschedpolicy(&attr, SCHED_SPORADIC);
    if (ret == 0) ret = pthread_attr_setschedparam(&attr, &param);
    if (ret == 0) ret = pthread_create(&scheduler_thread, &attr, scheduler_worker, tasks);
    pthread_attr_destroy(&attr);
    return ret;
}
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
    const char *ids[TASK_COUNT] = {"t1", "t2", "t3", "t4", "t5"};
    const char *names[TASK_COUNT] = {"Order Assignment", "Route Planning", "Item Pickup", "Goods Transportation", "Delivery Confirmation"};
    const unsigned int budgets[TASK_COUNT] = {T1_BUDGET_MS, T2_BUDGET_MS, T3_BUDGET_MS, T4_BUDGET_MS, T5_BUDGET_MS};
    const unsigned int workloads[TASK_COUNT] = {30, 40, 50, 60, 30};
    for (int i = 0; i < TASK_COUNT; i++) {
        strcpy(tasks[i].taskid, ids[i]);
        tasks[i].task_name = names[i];
        tasks[i].priority = task_priority(ids[i]);
        tasks[i].status = TASK_WAITING;
        tasks[i].timestamp = 0;
        tasks[i].budget_ms = budgets[i];
        tasks[i].remaining_budget_ms = budgets[i];
        tasks[i].remaining_work_ms = workloads[i];
        tasks[i].replenish_at_ns = 0;
        tasks[i].state = TASK_READY;
    }
}
static int get_timestamp_ms(uint64_t *timestamp) { // monotonic timestamp in milliseconds
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
static int send_watchdog_report(int watchdog_coid, int robot_id, uint32_t report_sequence, float front_distance, float left_distance, float right_distance, float speed, float acceleration, float x, float y) { // send simulated sensor and movement data to internal rover watchdog
    watchdog_message wmsg;
    uint64_t now;
    memset(&wmsg, 0, sizeof(wmsg));
    if (get_timestamp_ms(&now) == -1) return -1;
    wmsg.message_type = WATCHDOG_MSG_REPORT;
    wmsg.robot_id = robot_id;
    wmsg.report_sequence = report_sequence;
    wmsg.timestamp_ms = now;
    wmsg.front_distance = front_distance;
    wmsg.left_distance = left_distance;
    wmsg.right_distance = right_distance;
    wmsg.estimated_speed = speed;
    wmsg.acceleration_magnitude = acceleration;
    wmsg.position.x = x;
    wmsg.position.y = y;
    wmsg.position.heading = 0.0f;
    wmsg.sensors[SENSOR_FRONT_ULTRASONIC].sensor_id = SENSOR_FRONT_ULTRASONIC; // front ultrasonic
    wmsg.sensors[SENSOR_FRONT_ULTRASONIC].valid = SENSOR_VALID;
    wmsg.sensors[SENSOR_FRONT_ULTRASONIC].sequence = report_sequence;
    wmsg.sensors[SENSOR_FRONT_ULTRASONIC].timestamp_ms = now;
    wmsg.sensors[SENSOR_FRONT_ULTRASONIC].value = front_distance;
    wmsg.sensors[SENSOR_LEFT_ULTRASONIC].sensor_id = SENSOR_LEFT_ULTRASONIC; // left ultrasonic
    wmsg.sensors[SENSOR_LEFT_ULTRASONIC].valid = SENSOR_VALID;
    wmsg.sensors[SENSOR_LEFT_ULTRASONIC].sequence = report_sequence;
    wmsg.sensors[SENSOR_LEFT_ULTRASONIC].timestamp_ms = now;
    wmsg.sensors[SENSOR_LEFT_ULTRASONIC].value = left_distance;
    wmsg.sensors[SENSOR_RIGHT_ULTRASONIC].sensor_id = SENSOR_RIGHT_ULTRASONIC; // right ultrasonic
    wmsg.sensors[SENSOR_RIGHT_ULTRASONIC].valid = SENSOR_VALID;
    wmsg.sensors[SENSOR_RIGHT_ULTRASONIC].sequence = report_sequence;
    wmsg.sensors[SENSOR_RIGHT_ULTRASONIC].timestamp_ms = now;
    wmsg.sensors[SENSOR_RIGHT_ULTRASONIC].value = right_distance;
    wmsg.sensors[SENSOR_ADXL345].sensor_id = SENSOR_ADXL345; // adxl345 accelerometer
    wmsg.sensors[SENSOR_ADXL345].valid = SENSOR_VALID;
    wmsg.sensors[SENSOR_ADXL345].sequence = report_sequence;
    wmsg.sensors[SENSOR_ADXL345].timestamp_ms = now;
    wmsg.sensors[SENSOR_ADXL345].value = acceleration;
    if (MsgSend(watchdog_coid, &wmsg, sizeof(wmsg), NULL, 0) == -1) { // send report to internal watchdog
        perror("Internal Watchdog MsgSend failed");
        return -1;
    }
    printf("\n[ROVER %d] Internal watchdog report sent", robot_id);
    printf("\n[ROVER %d] Sensor sequence: %u", robot_id, report_sequence);
    return 0;
}
int main(int argc, char *argv[]) { // main rover process
    int coid = -1;
    int watchdog_coid = -1;
    int fw_coid = -1;
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
    uint32_t fw_sequence = 0;
    task robot_tasks[5];
    if (argc != 2) { // validate command-line arguments
        printf("Usage: rover <robot_id>\n");
        return EXIT_FAILURE;
    }
    id = atoi(argv[1]);
    if (id != 1 && id != 2) {
        printf("Invalid Robot ID. Use 1 or 2.\n");
        return EXIT_FAILURE;
    }
    initialise_task(robot_tasks);
    int sched_ret = start_scheduler(robot_tasks);
    if (sched_ret != 0) {
        fprintf(stderr, "[SCHEDULER] Failed to start worker thread: %d\n", sched_ret);
        return EXIT_FAILURE;
    }
    printf("[SCHEDULER] Worker thread started successfully.\n");
    printf("\n========== FLEETSHIELD ROVER ==========\n");
    printf("Robot ID: %d\n", id);
    printf("Process ID: %ld\n", (long)getpid());
    coid = name_open(SERVER_NAME, 0); // connect to fleet manager
    if (coid == -1) {
        perror("Fleet Manager name_open failed");
        scheduler_running = 0;
        pthread_join(scheduler_thread, NULL);
        return EXIT_FAILURE;
    }
    printf("Robot %d connected to Fleet Manager\n", id);
    char watchdog_name[40]; // connect to internal rover watchdog
    snprintf(watchdog_name, sizeof(watchdog_name), "rover_watchdog_%d", id);
    watchdog_coid = name_open(watchdog_name, 0);
    if (watchdog_coid == -1) {
        perror("Internal Watchdog name_open failed");
        name_close(coid);
        scheduler_running = 0;
        pthread_join(scheduler_thread, NULL);
        return EXIT_FAILURE;
    }
    printf("Robot %d connected to Internal Watchdog\n", id);
    fw_coid = fw_client_init(); // connect to fleet watchdog
    if (fw_coid == -1) {
        perror("Fleet Watchdog name_open failed");
        name_close(watchdog_coid);
        name_close(coid);
        scheduler_running = 0;
        pthread_join(scheduler_thread, NULL);
        return EXIT_FAILURE;
    }
    printf("Robot %d connected to Fleet Watchdog\n", id);
    if (id == 1) { // initialize robot paths
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
    while (1) { // main communication and movement loop (runs continuously so fleet watchdog can monitor health)
        robot_message msg = {0};
        robot_reply reply = {0};
        msg.type = MSG_ROBOT_REPORT; // message identification
        msg.robot_id = id;
        msg.source_x = source_x; // robot path information
        msg.source_y = source_y;
        msg.destination_x = destination_x;
        msg.destination_y = destination_y;
        msg.current_x = current_x;
        msg.current_y = current_y;
        msg.speed = current_speed; // movement information
        msg.priority = priority;
        msg.status = current_status;
        msg.scenario = SCENARIO_JUNCTION; // scenario information
        msg.segment_id = 101;
        msg.direction = (id == 1) ? 1 : 2;
        msg.segment_occupied = 0;
        msg.segment_clear = 0;
        sensor simulated_sensor = {0}; // simulated sensor values
        simulated_sensor.front_dist = 100.0f;
        simulated_sensor.left_dist = 100.0f;
        simulated_sensor.right_dist = 100.0f;
        simulated_sensor.accel_x = 0.0f;
        simulated_sensor.accel_y = 0.0f;
        simulated_sensor.accel_z = 0.0f;
        simulated_sensor.obstacle_detected = 0;
        msg.sensor_distance = simulated_sensor.front_dist;
        msg.acceleration_magnitude = 0.0f;
        if (get_timestamp_ms(&msg.timestamp_ms) == -1) { // timestamp and heartbeat
            current_status = STATUS_STOPPED;
            current_speed = 0.0f;
            break;
        }
        msg.sequence = sequence;
        msg.heartbeat_count = ++heartbeat_count;
        printf("\n------------------------------------"); // send report to internal watchdog
        printf("\nROVER %d: Sending internal watchdog report", id);
        if (send_watchdog_report(watchdog_coid, id, sequence, simulated_sensor.front_dist, simulated_sensor.left_dist, simulated_sensor.right_dist, current_speed, 0.0f, current_x, current_y) == -1) {
            printf("\nInternal watchdog communication failed.\n");
        }
        printf("\nSending report to Fleet Manager\n"); // send report to fleet manager
        int rc = MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply));
        if (rc == -1) {
            perror("Fleet Manager MsgSend failed");
            current_status = STATUS_STOPPED;
            current_speed = 0.0f;
            break;
        }
        int fw_result = fw_client_heartbeat(fw_coid, (id == 1) ? FW_ROLE_ROVER_1 : FW_ROLE_ROVER_2, ++fw_sequence); // send independent fleet heartbeat after manager communication
        if (fw_result != FW_ACK_OK) {
            printf("\n[ROVER %d] Fleet Watchdog heartbeat failed\n", id);
        }
        printf("\n========== ROBOT %d REPORT ==========\n", id); // display robot information
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
            printf("Manager: STOP - Robot stopped\n"); // rover remains alive and continues sending heartbeats
            current_status = STATUS_STOPPED;
            current_speed = 0.0f;
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
        sequence++; // increment report sequence
        sleep(1); // periodic report interval
    }
    printf("\nRobot %d terminating.\n", id); // cleanup
    fw_client_close(fw_coid);
    name_close(watchdog_coid);
    name_close(coid);
    scheduler_running = 0; // stop scheduler worker
    pthread_join(scheduler_thread, NULL);
    printf("[SCHEDULER] Worker thread stopped.\n");
    return EXIT_SUCCESS;
}
