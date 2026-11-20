#include <stdio.h>
#include <float.h>
#include "fleet_common.h"
#include "fleet_junction.h"
static int fleet_isfinite(float value) { // local math helpers with no external libm dependency
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX);
}
static float fleet_absf(float value) {
    return (value < 0.0f) ? -value : value;
}
static float fleet_sqrtf(float value) {
    if (!fleet_isfinite(value) || value < 0.0f) return -1.0f;
    if (value == 0.0f) return 0.0f;
    float estimate = (value >= 1.0f) ? value : 1.0f;
    for (int i = 0; i < 24; i++) {
        estimate = 0.5f * (estimate + value / estimate);
    }
    return estimate;
}
static int valid_rover_data(const junction_rover_data *r) { // validate rover data
    if (r == NULL || !r->sensor_valid) return 0;
    if (!fleet_isfinite(r->x) || !fleet_isfinite(r->y) || !fleet_isfinite(r->vx) || !fleet_isfinite(r->vy) || !fleet_isfinite(r->front_distance) || !fleet_isfinite(r->left_distance) || !fleet_isfinite(r->right_distance) || !fleet_isfinite(r->acceleration_x) || !fleet_isfinite(r->acceleration_y) || !fleet_isfinite(r->acceleration_z)) {
        return 0;
    }
    if (r->front_distance < 0.0f || r->left_distance < 0.0f || r->right_distance < 0.0f) {
        return 0;
    }
    return 1;
}
static float calculate_junction_eta(const junction_rover_data *r) { // calculate eta to junction
    float speed = fleet_sqrtf(r->vx * r->vx + r->vy * r->vy);
    if (speed < 0.001f) return -1.0f;
    float dx = JUNCTION_X_M - r->x;
    float dy = JUNCTION_Y_M - r->y;
    float distance = (dx * r->vx + dy * r->vy) / speed; // project distance along direction of travel
    if (distance <= 0.0f) return -1.0f;
    float lateral_distance = fleet_absf(dx * r->vy - dy * r->vx) / speed; // check whether trajectory approaches junction
    if (lateral_distance > 0.20f) return -1.0f;
    return distance / speed;
}
static float calculate_minimum_separation(const junction_rover_data *r1, const junction_rover_data *r2) { // calculate minimum predicted separation
    float rx = r2->x - r1->x;
    float ry = r2->y - r1->y;
    float vx = r2->vx - r1->vx;
    float vy = r2->vy - r1->vy;
    float velocity_squared = vx * vx + vy * vy;
    if (velocity_squared < 0.000001f) { // relative velocity is almost zero
        return fleet_sqrtf(rx * rx + ry * ry);
    }
    float t_closest = -(rx * vx + ry * vy) / velocity_squared; // time of closest approach
    if (t_closest < 0.0f) t_closest = 0.0f;
    float closest_x = rx + vx * t_closest; // relative position at closest approach
    float closest_y = ry + vy * t_closest;
    return fleet_sqrtf(closest_x * closest_x + closest_y * closest_y);
}
int junction_collision_check(const junction_rover_data *robot1, const junction_rover_data *robot2, junction_result *result) { // junction collision check
    if (result == NULL) return -1;
    result->collision_detected = 0; // default to safe commands
    result->command_robot1 = CMD_STOP;
    result->command_robot2 = CMD_STOP;
    result->eta_robot1 = -1.0f;
    result->eta_robot2 = -1.0f;
    result->minimum_predicted_separation = -1.0f;
    if (!valid_rover_data(robot1) || !valid_rover_data(robot2)) { // validate both rover reports
        printf("\n[JUNCTION] Invalid sensor/motion data\n");
        printf("[JUNCTION] SAFE STOP REQUIRED\n");
        return -1;
    }
    float eta1 = calculate_junction_eta(robot1); // calculate junction eta
    float eta2 = calculate_junction_eta(robot2);
    result->eta_robot1 = eta1;
    result->eta_robot2 = eta2;
    printf("\n================================\n");
    printf(" FLEETSHIELD JUNCTION ANALYSIS\n");
    printf("================================\n");
    printf("Robot 1 ETA: %.2f seconds\n", eta1);
    printf("Robot 2 ETA: %.2f seconds\n", eta2);
    if (robot1->front_distance <= ULTRASONIC_STOP_M) { // ultrasonic obstacle check for robot 1
        result->command_robot1 = CMD_STOP;
        printf("[SENSOR] Robot 1 FRONT STOP\n");
    } else if (robot1->front_distance <= ULTRASONIC_WARNING_M) {
        result->command_robot1 = CMD_SLOW;
        printf("[SENSOR] Robot 1 FRONT WARNING\n");
    } else {
        result->command_robot1 = CMD_MOVE;
    }
    if (robot2->front_distance <= ULTRASONIC_STOP_M) { // ultrasonic obstacle check for robot 2
        result->command_robot2 = CMD_STOP;
        printf("[SENSOR] Robot 2 FRONT STOP\n");
    } else if (robot2->front_distance <= ULTRASONIC_WARNING_M) {
        result->command_robot2 = CMD_SLOW;
        printf("[SENSOR] Robot 2 FRONT WARNING\n");
    } else {
        result->command_robot2 = CMD_MOVE;
    }
    result->minimum_predicted_separation = calculate_minimum_separation(robot1, robot2); // relative motion prediction
    if (result->minimum_predicted_separation < 0.0f) {
        printf("[JUNCTION] Invalid separation calculation\n");
        return -1;
    }
    printf("Minimum predicted separation: %.2f m\n", result->minimum_predicted_separation);
    if (eta1 >= 0.0f && eta2 >= 0.0f) { // occupancy window calculation
        float speed1 = fleet_sqrtf(robot1->vx * robot1->vx + robot1->vy * robot1->vy);
        float speed2 = fleet_sqrtf(robot2->vx * robot2->vx + robot2->vy * robot2->vy);
        if (speed1 < 0.001f || speed2 < 0.001f) return -1;
        float exit1 = eta1 + JUNCTION_LENGTH_M / speed1 + JUNCTION_GUARD_TIME_S;
        float exit2 = eta2 + JUNCTION_LENGTH_M / speed2 + JUNCTION_GUARD_TIME_S;
        printf("\nOCCUPANCY WINDOWS\n");
        printf("Robot 1: [%.2f, %.2f]\n", eta1, exit1);
        printf("Robot 2: [%.2f, %.2f]\n", eta2, exit2);
        if (eta1 <= exit2 && eta2 <= exit1) { // check whether occupancy windows overlap
            result->collision_detected = 1;
            result->command_robot1 = CMD_STOP;
            result->command_robot2 = CMD_STOP;
            printf("\n[JUNCTION] CONFLICT DETECTED!\n");
        } else {
            printf("\n[JUNCTION] No occupancy-window overlap\n");
        }
    } else {
        printf("\n[JUNCTION] Both rovers are not approaching the conflict point\n");
    }
    printf("\nFINAL COMMANDS\n"); // final decision
    printf("Robot 1: %d\n", result->command_robot1);
    printf("Robot 2: %d\n", result->command_robot2);
    printf("Collision status: %s\n", result->collision_detected ? "DETECTED" : "NOT DETECTED");
    printf("================================\n");
    return 0;
}
