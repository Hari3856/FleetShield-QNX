
#ifndef FLEET_JUNCTION_H
#define FLEET_JUNCTION_H
#define JUNCTION_X_M 3.0f
#define JUNCTION_Y_M 2.0f

/* Junction collision configuration */
#define JUNCTION_LENGTH_M       0.8f
#define JUNCTION_GUARD_TIME_S   0.5f

#define ULTRASONIC_WARNING_M    0.8f
#define ULTRASONIC_STOP_M       0.4f

/* Rover sensor and motion sample */
typedef struct
{
    int robot_id;

    float x;
    float y;

    float vx;
    float vy;

    float front_distance;
    float left_distance;
    float right_distance;

    float acceleration_x;
    float acceleration_y;
    float acceleration_z;

    int sensor_valid;

} junction_rover_data;

/* Collision decision */
typedef struct
{
    int collision_detected;

    int command_robot1;
    int command_robot2;

    float eta_robot1;
    float eta_robot2;

    float minimum_predicted_separation;

} junction_result;

int junction_collision_check(
    const junction_rover_data *robot1,
    const junction_rover_data *robot2,
    junction_result *result
);

#endif
