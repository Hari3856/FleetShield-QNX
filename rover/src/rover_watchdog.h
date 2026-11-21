#ifndef ROVER_WATCHDOG_H
#define ROVER_WATCHDOG_H
#include <stdint.h>
#define SENSOR_COUNT 4 // rover watchdog configuration
#define SENSOR_FRONT_ULTRASONIC 0
#define SENSOR_LEFT_ULTRASONIC 1
#define SENSOR_RIGHT_ULTRASONIC 2
#define SENSOR_ADXL345 3
#define WATCHDOG_MSG_REPORT 1 // qnx ipc configuration
#define WATCHDOG_TIMER_PULSE 20
#define WATCHDOG_TIMER_INTERVAL_MS 500
#define SENSOR_TIMEOUT_MS 3000 // sensor heartbeat timeout
#define SENSOR_INVALID 0 // sensor status
#define SENSOR_VALID 1
#define SAFETY_NORMAL 0 // safety states
#define SAFETY_FAULT 1
#define SAFETY_SAFE_STOP 2
#define FAULT_NONE 0 // fault classifications
#define FAULT_SENSOR_TIMEOUT 1
#define FAULT_INVALID_SENSOR 2
#define FAULT_ABNORMAL_DISTANCE 3
#define FAULT_SPEED_LIMIT 4
#define FAULT_ACCELERATION_LIMIT 5
#define FAULT_POSITION_INVALID 6
typedef struct { // sensor report structure
    int sensor_id;
    int valid;
    uint32_t sequence;
    uint64_t timestamp_ms;
    float value;
} sensor_report;
typedef struct { // rover coordinate structure
    float x;
    float y;
    float heading;
} rover_position;
typedef struct { // rover watchdog message
    int message_type;
    int robot_id;
    sensor_report sensors[SENSOR_COUNT]; // sensor reports
    float front_distance; // distance measurements in metres
    float left_distance;
    float right_distance;
    float estimated_speed; // movement info
    float acceleration_magnitude;
    rover_position position; // rover coordinates
    uint32_t report_sequence; // report sequence
    uint64_t timestamp_ms; // rover report timestamp
} watchdog_message;
typedef struct { // watchdog status
    int safety_state;
    int fault_code;
    int faulty_sensor_id;
} watchdog_status;
#endif
