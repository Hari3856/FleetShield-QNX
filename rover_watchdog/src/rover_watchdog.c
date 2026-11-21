#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <float.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include "rover_watchdog.h"
typedef struct { // sensor health tracking
    int seen;
    uint32_t last_sequence;
    uint64_t last_timestamp_ms;
} sensor_health;
static sensor_health health[SENSOR_COUNT];
static int robot_id; // watchdog global state
static int current_fault = FAULT_NONE;
static int faulty_sensor_id = -1;
static uint64_t watchdog_start_ms;
static int is_finite_float(float value) { // get monotonic time in ms
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX);
}
static int get_timestamp_ms(uint64_t *timestamp) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1) {
        perror("clock_gettime");
        return -1;
    }
    *timestamp = ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
    return 0;
}
static void report_fault(int fault, int sensor_id) { // report detected fault (monitoring only, no motor control or recovery here)
    current_fault = fault;
    faulty_sensor_id = sensor_id;
    printf("\n====================================\n");
    printf("[WATCHDOG] FAULT DETECTED\n");
    printf("Robot ID: %d\n", robot_id);
    printf("Fault code: %d\n", fault);
    if (sensor_id >= 0) printf("Faulty sensor ID: %d\n", sensor_id);
    printf("====================================\n");
}
static int validate_sensors(const watchdog_message *msg, uint64_t now) { // validate sensor reports
    int i;
    for (i = 0; i < SENSOR_COUNT; i++) {
        const sensor_report *s = &msg->sensors[i];
        if (s->sensor_id != i) { // check sensor identity
            report_fault(FAULT_INVALID_SENSOR, i);
            return -1;
        }
        if (s->valid != SENSOR_VALID) { // check sensor validity
            report_fault(FAULT_INVALID_SENSOR, i);
            return -1;
        }
        if (!is_finite_float(s->value)) { // check reading
            report_fault(FAULT_INVALID_SENSOR, i);
            return -1;
        }
        if (s->timestamp_ms == 0 || s->timestamp_ms > now) { // check timestamp
            report_fault(FAULT_INVALID_SENSOR, i);
            return -1;
        }
        if (health[i].seen && s->sequence <= health[i].last_sequence) { // check sequence progression
            report_fault(FAULT_INVALID_SENSOR, i);
            return -1;
        }
        health[i].seen = 1; // store latest sensor info
        health[i].last_sequence = s->sequence;
        health[i].last_timestamp_ms = s->timestamp_ms;
    }
    return 0;
}
static int validate_movement(const watchdog_message *msg) { // validate rover movement info
    if (!is_finite_float(msg->estimated_speed) || msg->estimated_speed < 0.0f) { // validate speed
        report_fault(FAULT_SPEED_LIMIT, -1);
        return -1;
    }
    if (!is_finite_float(msg->acceleration_magnitude) || msg->acceleration_magnitude < 0.0f) { // validate acceleration
        report_fault(FAULT_ACCELERATION_LIMIT, -1);
        return -1;
    }
    return 0;
}
static int validate_distances(const watchdog_message *msg) { // validate distance measurements
    if (!is_finite_float(msg->front_distance) || !is_finite_float(msg->left_distance) || !is_finite_float(msg->right_distance)) {
        report_fault(FAULT_ABNORMAL_DISTANCE, -1);
        return -1;
    }
    if (msg->front_distance < 0.0f || msg->left_distance < 0.0f || msg->right_distance < 0.0f) {
        report_fault(FAULT_ABNORMAL_DISTANCE, -1);
        return -1;
    }
    return 0;
}
static int validate_position(const watchdog_message *msg) { // validate rover coordinates
    if (!is_finite_float(msg->position.x) || !is_finite_float(msg->position.y) || !is_finite_float(msg->position.heading)) {
        report_fault(FAULT_POSITION_INVALID, -1);
        return -1;
    }
    return 0;
}
static void process_rover_report(const watchdog_message *msg) { // process rover report
    uint64_t now;
    if (msg->robot_id != robot_id) { // check robot identity
        printf("[WATCHDOG] Incorrect robot ID. Ignored.\n");
        return;
    }
    if (get_timestamp_ms(&now) == -1) return;
    if (msg->timestamp_ms == 0 || msg->timestamp_ms > now || now - msg->timestamp_ms > SENSOR_TIMEOUT_MS) { // validate report timestamp
        report_fault(FAULT_INVALID_SENSOR, -1);
        return;
    }
    if (validate_sensors(msg, now) == -1) return; // validate individual sensors
    if (validate_movement(msg) == -1) return; // validate speed and acceleration
    if (validate_distances(msg) == -1) return; // validate obstacle distances
    if (validate_position(msg) == -1) return; // validate coordinates
    printf("\n========== WATCHDOG REPORT ==========\n"); // display received info
    printf("Robot ID: %d\n", msg->robot_id);
    printf("Report sequence: %u\n", msg->report_sequence);
    printf("\n--- SENSOR DISTANCES ---\n");
    printf("Front: %.2f m\n", msg->front_distance);
    printf("Left: %.2f m\n", msg->left_distance);
    printf("Right: %.2f m\n", msg->right_distance);
    printf("\n--- MOVEMENT INFORMATION ---\n");
    printf("Speed: %.2f\n", msg->estimated_speed);
    printf("Acceleration: %.2f\n", msg->acceleration_magnitude);
    printf("\n--- ROVER POSITION ---\n");
    printf("X: %.2f\n", msg->position.x);
    printf("Y: %.2f\n", msg->position.y);
    printf("Heading: %.2f\n", msg->position.heading);
    printf("\nAll received values passed basic validation.\n");
    printf("====================================\n");
}
static void check_sensor_health(void) { // periodic sensor heartbeat check
    int i;
    uint64_t now;
    if (get_timestamp_ms(&now) == -1) return;
    for (i = 0; i < SENSOR_COUNT; i++) {
        uint64_t last_time;
        if (!health[i].seen) { // if no report arrived yet, measure timeout from watchdog startup
            last_time = watchdog_start_ms;
        } else {
            last_time = health[i].last_timestamp_ms;
        }
        if (now >= last_time && (now - last_time) > SENSOR_TIMEOUT_MS) {
            report_fault(FAULT_SENSOR_TIMEOUT, i);
        }
    }
}
int rover_watchdog_main(int argc, char *argv[]) {
    name_attach_t *attach;
    int timer_coid;
    timer_t timer_id;
    struct sigevent event;
    struct _itimer timer_config;
    char watchdog_name[40];
    union {
        struct _pulse pulse;
        watchdog_message message;
    } received;
    if (argc != 2) { // validate command-line arg
        printf("Usage: rover_watchdog <robot_id>\n");
        return EXIT_FAILURE;
    }
    robot_id = atoi(argv[1]);
    if (robot_id != 1 && robot_id != 2) {
        printf("Invalid Robot ID. Use 1 or 2.\n");
        return EXIT_FAILURE;
    }
    memset(health, 0, sizeof(health)); // init watchdog
    if (get_timestamp_ms(&watchdog_start_ms) == -1) return EXIT_FAILURE;
    snprintf(watchdog_name, sizeof(watchdog_name), "rover_watchdog_%d", robot_id); // create unique watchdog name
    attach = name_attach(NULL, watchdog_name, 0); // create qnx communication channel
    if (attach == NULL) {
        perror("name_attach");
        return EXIT_FAILURE;
    }
    printf("\n====================================\n");
    printf("FLEETSHIELD INTERNAL WATCHDOG\n");
    printf("Robot ID: %d\n", robot_id);
    printf("Watchdog name: %s\n", watchdog_name);
    printf("Process ID: %ld\n", (long)getpid());
    printf("Monitoring four sensors\n");
    printf("====================================\n");
    timer_coid = ConnectAttach(0, getpid(), attach->chid, _NTO_SIDE_CHANNEL, 0); // connect timer to watchdog channel
    if (timer_coid == -1) {
        perror("ConnectAttach");
        name_detach(attach, 0);
        return EXIT_FAILURE;
    }
    SIGEV_PULSE_INIT(&event, timer_coid, SIGEV_PULSE_PRIO_INHERIT, WATCHDOG_TIMER_PULSE, 0); // configure timer pulse
    timer_id = TimerCreate(CLOCK_MONOTONIC, &event); // create qnx timer
    if (timer_id == -1) {
        perror("TimerCreate");
        ConnectDetach(timer_coid);
        name_detach(attach, 0);
        return EXIT_FAILURE;
    }
    memset(&timer_config, 0, sizeof(timer_config)); // configure periodic timer
    timer_config.nsec = (uint64_t)WATCHDOG_TIMER_INTERVAL_MS * 1000000ULL;
    timer_config.interval_nsec = (uint64_t)WATCHDOG_TIMER_INTERVAL_MS * 1000000ULL;
    if (TimerSettime(timer_id, 0, &timer_config, NULL) == -1) {
        perror("TimerSettime");
        TimerDestroy(timer_id);
        ConnectDetach(timer_coid);
        name_detach(attach, 0);
        return EXIT_FAILURE;
    }
    printf("[WATCHDOG] Timer started.\n");
    printf("[WATCHDOG] Monitoring interval: %d ms\n", WATCHDOG_TIMER_INTERVAL_MS);
    printf("[WATCHDOG] Sensor timeout: %d ms\n", SENSOR_TIMEOUT_MS);
    while (1) { // main message and pulse loop
        int rcvid;
        rcvid = MsgReceive(attach->chid, &received, sizeof(received), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            perror("MsgReceive");
            break;
        }
        if (rcvid == 0) { // handle timer pulse
            if (received.pulse.code == WATCHDOG_TIMER_PULSE) {
                check_sensor_health();
            }
            continue;
        }
        if (received.message.message_type == WATCHDOG_MSG_REPORT) { // handle normal rover message
            process_rover_report(&received.message);
            MsgReply(rcvid, 0, NULL, 0);
        } else {
            MsgError(rcvid, EINVAL);
        }
    }
    TimerDestroy(timer_id); // cleanup
    ConnectDetach(timer_coid);
    name_detach(attach, 0);
    return EXIT_SUCCESS;
}
