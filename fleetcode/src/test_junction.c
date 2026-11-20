#include <stdio.h>
#include <string.h>
#include "fleet_common.h"
#include "fleet_junction.h"

#define TOTAL_JUNCTION_TESTS 10

static int total_tests, passed_tests, failed_tests;
static int collision_events, stop_commands, slow_commands;
static int move_commands, invalid_sensor_events;

/* Local square root: no libm dependency. */
static float display_sqrt(float value)
{
    float estimate;
    int i;

    if (value <= 0.0f) return 0.0f;
    estimate = (value >= 1.0f) ? value : 1.0f;

    for (i = 0; i < 24; ++i)
        estimate = 0.5f * (estimate + value / estimate);

    return estimate;
}

static const char *command_name(int command)
{
    switch (command) {
        case CMD_STOP:   return "STOP";
        case CMD_SLOW:   return "SLOW";
        case CMD_MOVE:   return "MOVE";
        case CMD_RESUME: return "RESUME";
        default:         return "UNKNOWN";
    }
}

static void initialize_robots(junction_rover_data *r1,
                               junction_rover_data *r2)
{
    memset(r1, 0, sizeof(*r1));
    memset(r2, 0, sizeof(*r2));

    r1->robot_id = 1;
    r2->robot_id = 2;
    r1->sensor_valid = 1;
    r2->sensor_valid = 1;

    r1->front_distance = 2.0f;
    r1->left_distance = 1.5f;
    r1->right_distance = 1.5f;
    r2->front_distance = 2.0f;
    r2->left_distance = 1.5f;
    r2->right_distance = 1.5f;
}

static void display_robot(const junction_rover_data *r)
{
    float speed_squared = r->vx * r->vx + r->vy * r->vy;

    printf("\nROBOT %d INPUT TELEMETRY\n", r->robot_id);
    printf("-----------------------------------------------\n");
    printf("Position             : (%.2f, %.2f) m\n", r->x, r->y);
    printf("Velocity             : VX %.2f | VY %.2f m/s\n",
           r->vx, r->vy);
    printf("Speed                : %.2f m/s\n",
           display_sqrt(speed_squared));
    printf("Front ultrasonic     : %.2f m\n", r->front_distance);
    printf("Left ultrasonic      : %.2f m\n", r->left_distance);
    printf("Right ultrasonic     : %.2f m\n", r->right_distance);
    printf("Acceleration         : X %.2f | Y %.2f | Z %.2f m/s^2\n",
           r->acceleration_x, r->acceleration_y, r->acceleration_z);
    printf("Sensor status        : %s\n",
           r->sensor_valid ? "VALID" : "INVALID");
}

static void record_command(int command)
{
    if (command == CMD_STOP) ++stop_commands;
    else if (command == CMD_SLOW) ++slow_commands;
    else if (command == CMD_MOVE) ++move_commands;
}

static void run_test(
    const char *test_id,
    const char *description,
    junction_rover_data *r1,
    junction_rover_data *r2,
    int expect_invalid,
    int expected_collision,
    int expected_command1,
    int expected_command2)
{
    junction_result result;
    int status;
    int pass = 0;

    ++total_tests;

    printf("\n\n============================================================\n");
    printf(" FLEETSHIELD | SCENARIO 01 | PERPENDICULAR JUNCTION\n");
    printf(" TEST ID : %s\n TEST    : %s\n", test_id, description);
    printf("============================================================\n");

    printf("\n[1] INPUT TELEMETRY\n");
    display_robot(r1);
    display_robot(r2);

    printf("\n[2] COLLISION ENGINE EXECUTION\n");
    printf("-----------------------------------------------\n");

    status = junction_collision_check(r1, r2, &result);

    if (expect_invalid) {
        printf("\n[3] FAIL-SAFE VALIDATION\n");
        if (status != 0 &&
            result.command_robot1 == CMD_STOP &&
            result.command_robot2 == CMD_STOP) {
            printf("Invalid data rejected : YES\n");
            printf("Robot 1 safe command  : STOP\n");
            printf("Robot 2 safe command  : STOP\n");
            pass = 1;
            ++invalid_sensor_events;
            stop_commands += 2;
        } else {
            printf("Invalid data rejection / safe-stop verification FAILED\n");
        }
    } else if (status != 0) {
        printf("\n[ERROR] Collision function returned status %d\n", status);
    } else {
        printf("\n[3] PREDICTION RESULTS\n");
        printf("-----------------------------------------------\n");
        printf("Robot 1 junction ETA          : %.2f s\n", result.eta_robot1);
        printf("Robot 2 junction ETA          : %.2f s\n", result.eta_robot2);
        printf("Minimum predicted separation  : %.2f m\n",
               result.minimum_predicted_separation);
        printf("Junction conflict             : %s\n",
               result.collision_detected ? "DETECTED" : "NOT DETECTED");

        printf("\n[4] EXPECTED VS ACTUAL DECISION\n");
        printf("-----------------------------------------------\n");
        printf("%-24s %-15s %-15s\n", "PARAMETER", "EXPECTED", "ACTUAL");
        printf("------------------------------------------------------------\n");
        printf("%-24s %-15s %-15s\n", "Collision",
               expected_collision ? "DETECTED" : "CLEAR",
               result.collision_detected ? "DETECTED" : "CLEAR");
        printf("%-24s %-15s %-15s\n", "Robot 1 command",
               command_name(expected_command1),
               command_name(result.command_robot1));
        printf("%-24s %-15s %-15s\n", "Robot 2 command",
               command_name(expected_command2),
               command_name(result.command_robot2));

        pass = (result.collision_detected == expected_collision) &&
               (result.command_robot1 == expected_command1) &&
               (result.command_robot2 == expected_command2);

        if (result.collision_detected) ++collision_events;
        record_command(result.command_robot1);
        record_command(result.command_robot2);
    }

    printf("\n[5] TEST VERDICT\n");
    printf("-----------------------------------------------\n");
    if (pass) {
        ++passed_tests;
        printf("RESULT : PASS\nDecision matches expected behaviour.\n");
    } else {
        ++failed_tests;
        printf("RESULT : FAIL\nReview inputs and collision algorithm.\n");
    }
    printf("============================================================\n");
}

/* J01: ETA1=2.00 s, ETA2=2.00 s. */
static void test_J01(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=1.8f; r1.y=2.0f; r1.vx=0.60f; r1.vy=0.0f;
    r2.x=3.0f; r2.y=0.8f; r2.vx=0.0f; r2.vy=0.60f;
    r1.front_distance=1.8f; r1.left_distance=1.2f; r1.right_distance=1.4f;
    r2.front_distance=1.7f; r2.left_distance=1.3f; r2.right_distance=1.1f;
    r1.acceleration_x=0.12f; r2.acceleration_y=0.08f;
    run_test("J01","SIMULTANEOUS JUNCTION ARRIVAL",
             &r1,&r2,0,1,CMD_STOP,CMD_STOP);
}

/* J02: ETA1=2.00 s, ETA2=2.30 s. */
static void test_J02(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=2.1f; r1.y=2.0f; r1.vx=0.45f; r1.vy=0.0f;
    r2.x=3.0f; r2.y=0.62f; r2.vx=0.0f; r2.vy=0.60f;
    r1.front_distance=1.1f; r1.left_distance=0.9f; r1.right_distance=1.5f;
    r2.front_distance=1.6f; r2.left_distance=1.2f; r2.right_distance=1.4f;
    r1.acceleration_x=0.18f; r2.acceleration_y=0.05f;
    run_test("J02","ROBOT 1 ARRIVES BEFORE ROBOT 2",
             &r1,&r2,0,1,CMD_STOP,CMD_STOP);
}

/* J03: ETA1=3.00 s, ETA2=2.00 s. */
static void test_J03(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=1.5f; r1.y=2.0f; r1.vx=0.50f; r1.vy=0.0f;
    r2.x=3.0f; r2.y=1.2f; r2.vx=0.0f; r2.vy=0.40f;
    r1.front_distance=2.2f; r1.left_distance=1.6f; r1.right_distance=1.1f;
    r2.front_distance=1.3f; r2.left_distance=1.5f; r2.right_distance=0.9f;
    r1.acceleration_x=-0.06f; r2.acceleration_y=0.14f;
    run_test("J03","ROBOT 2 ARRIVES BEFORE ROBOT 1",
             &r1,&r2,0,1,CMD_STOP,CMD_STOP);
}

/* J04: ETA1=1.50 s, ETA2=6.00 s; occupancy windows do not overlap. */
static void test_J04(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=2.4f; r1.y=2.0f; r1.vx=0.40f; r1.vy=0.0f;
    r2.x=3.0f; r2.y=-1.0f; r2.vx=0.0f; r2.vy=0.50f;
    r1.front_distance=1.5f; r1.left_distance=1.1f; r1.right_distance=1.8f;
    r2.front_distance=2.0f; r2.left_distance=1.4f; r2.right_distance=1.2f;
    r1.acceleration_x=0.04f; r2.acceleration_y=0.09f;
    run_test("J04","SAFE STAGGERED JUNCTION ARRIVAL",
             &r1,&r2,0,0,CMD_MOVE,CMD_MOVE);
}

/* J05: ETA1=4.00 s, ETA2=4.50 s. */
static void test_J05(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=2.2f; r1.y=2.0f; r1.vx=0.20f; r1.vy=0.0f;
    r2.x=3.0f; r2.y=1.1f; r2.vx=0.0f; r2.vy=0.20f;
    r1.front_distance=0.95f; r1.left_distance=0.8f; r1.right_distance=1.0f;
    r2.front_distance=1.2f; r2.left_distance=0.9f; r2.right_distance=1.1f;
    r1.acceleration_x=0.02f; r2.acceleration_y=0.03f;
    run_test("J05","LOW SPEED JUNCTION APPROACH",
             &r1,&r2,0,1,CMD_STOP,CMD_STOP);
}

/* J06: ETA1=2.00 s, ETA2=2.00 s. */
static void test_J06(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=1.0f; r1.y=2.0f; r1.vx=1.00f; r1.vy=0.0f;
    r2.x=3.0f; r2.y=0.5f; r2.vx=0.0f; r2.vy=0.75f;
    r1.front_distance=1.9f; r1.left_distance=1.5f; r1.right_distance=1.3f;
    r2.front_distance=1.8f; r2.left_distance=1.1f; r2.right_distance=1.6f;
    r1.acceleration_x=0.25f; r2.acceleration_y=0.20f;
    run_test("J06","HIGH SPEED JUNCTION APPROACH",
             &r1,&r2,0,1,CMD_STOP,CMD_STOP);
}

/* J07: ETA1=1.00 s, ETA2=2.50 s; occupancy windows overlap. */
static void test_J07(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=2.7f; r1.y=2.0f; r1.vx=0.30f; r1.vy=0.0f;
    r2.x=3.0f; r2.y=1.125f; r2.vx=0.0f; r2.vy=0.35f;
    r1.front_distance=1.4f; r1.left_distance=0.7f; r1.right_distance=1.2f;
    r2.front_distance=0.85f; r2.left_distance=1.0f; r2.right_distance=0.8f;
    r1.acceleration_x=-0.10f; r2.acceleration_y=0.16f;
    run_test("J07","DIFFERENT SPEEDS AND OVERLAPPING OCCUPANCY",
             &r1,&r2,0,1,CMD_STOP,CMD_STOP);
}

/* J08: isolated front obstacle; R1 STOP, R2 MOVE. */
static void test_J08(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=0.8f; r1.y=4.5f; r1.vx=0.15f; r1.vy=0.05f;
    r2.x=8.5f; r2.y=-0.5f; r2.vx=0.0f; r2.vy=0.0f;
    r1.front_distance=0.18f; r1.left_distance=0.55f; r1.right_distance=0.92f;
    r2.front_distance=1.65f; r2.left_distance=1.3f; r2.right_distance=1.8f;
    r1.acceleration_x=-0.22f; r1.acceleration_y=0.04f;
    run_test("J08","ROBOT 1 CRITICAL FRONT OBSTACLE",
             &r1,&r2,0,0,CMD_STOP,CMD_MOVE);
}

/* J09: R1 warning, R2 critical obstacle. */
static void test_J09(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=1.4f; r1.y=4.2f; r1.vx=0.20f; r1.vy=-0.10f;
    r2.x=8.0f; r2.y=4.5f; r2.vx=-0.10f; r2.vy=0.05f;
    r1.front_distance=0.72f; r1.left_distance=0.48f; r1.right_distance=1.25f;
    r2.front_distance=0.32f; r2.left_distance=0.68f; r2.right_distance=0.44f;
    r1.acceleration_x=0.07f; r1.acceleration_y=-0.03f;
    r2.acceleration_x=-0.12f; r2.acceleration_y=0.06f;
    run_test("J09","ROBOT 1 WARNING AND ROBOT 2 CRITICAL OBSTACLE",
             &r1,&r2,0,0,CMD_SLOW,CMD_STOP);
}

/* J10: invalid report; junction function initializes both commands to STOP. */
static void test_J10(void)
{
    junction_rover_data r1, r2;
    initialize_robots(&r1, &r2);
    r1.x=1.65f; r1.y=2.0f; r1.vx=0.55f; r1.vy=0.0f;
    r2.x=3.0f; r2.y=0.95f; r2.vx=0.0f; r2.vy=0.45f;
    r1.front_distance=1.35f; r1.left_distance=0.88f; r1.right_distance=1.42f;
    r2.front_distance=1.15f; r2.left_distance=0.76f; r2.right_distance=1.32f;
    r1.acceleration_x=0.11f; r2.acceleration_y=-0.08f;
    r2.sensor_valid=0;
    run_test("J10","ROBOT 2 INVALID SENSOR REPORT",
             &r1,&r2,1,0,CMD_STOP,CMD_STOP);
}

static void display_summary(void)
{
    printf("\n\n############################################################\n");
    printf("             FLEETSHIELD TEST SUMMARY\n");
    printf("############################################################\n");
    printf("\nSCENARIO : PERPENDICULAR JUNCTION\n");

    printf("\nTEST EXECUTION\n");
    printf("-----------------------------------------------\n");
    printf("Total tests              : %d\n", total_tests);
    printf("Passed                   : %d\n", passed_tests);
    printf("Failed                   : %d\n", failed_tests);

    printf("\nSAFETY EVENTS\n");
    printf("-----------------------------------------------\n");
    printf("Junction conflicts       : %d\n", collision_events);
    printf("STOP commands            : %d\n", stop_commands);
    printf("SLOW commands            : %d\n", slow_commands);
    printf("MOVE commands            : %d\n", move_commands);
    printf("Invalid sensor tests     : %d\n", invalid_sensor_events);

    printf("\nOVERALL VALIDATION\n");
    printf("-----------------------------------------------\n");
    if (total_tests == TOTAL_JUNCTION_TESTS && failed_tests == 0)
        printf("RESULT : ALL TESTS PASSED\n");
    else
        printf("RESULT : VALIDATION FAILED - REVIEW REQUIRED\n");

    printf("############################################################\n");
    printf("        JUNCTION TEST EXECUTION COMPLETED\n");
    printf("############################################################\n");
}

int main(void)
{
    printf("\n############################################################\n");
    printf("                  FLEETSHIELD\n");
    printf("       AUTONOMOUS ROBOT FLEET SAFETY SYSTEM\n");
    printf("############################################################\n");
    printf("MODULE       : COLLISION DETECTION\n");
    printf("SCENARIO     : 01 - PERPENDICULAR JUNCTION\n");
    printf("INPUT SOURCE : SIMULATED ROVER TELEMETRY\n");
    printf("TEST CASES   : %d\n", TOTAL_JUNCTION_TESTS);

    test_J01();
    test_J02();
    test_J03();
    test_J04();
    test_J05();
    test_J06();
    test_J07();
    test_J08();
    test_J09();
    test_J10();

    display_summary();

    return (failed_tests == 0 &&
            total_tests == TOTAL_JUNCTION_TESTS) ? 0 : 1;
}
