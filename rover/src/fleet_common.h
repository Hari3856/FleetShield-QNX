
#ifndef FLEET_COMMON_H
#define FLEET_COMMON_H

#include <stdint.h>

/* ---------------------------------------
   GENERAL CONFIGURATION
--------------------------------------- */

#define SERVER_NAME "fleet_manager"

#define ROBOT_COUNT 2

#define HEARTBEAT_INTERVAL_MS 1000
#define HEARTBEAT_TIMEOUT_MS 3000


/* ---------------------------------------
   MESSAGE TYPES
--------------------------------------- */

#define MSG_ROBOT_REPORT 1


/* ---------------------------------------
   MANAGER COMMANDS
--------------------------------------- */

#define CMD_STOP   1
#define CMD_SLOW   2
#define CMD_MOVE   3
#define CMD_RESUME 4


/* ---------------------------------------
   ROBOT STATUS
--------------------------------------- */

#define STATUS_MOVING  1
#define STATUS_SLOWING 2
#define STATUS_STOPPED 3


/* ---------------------------------------
   COLLISION SCENARIOS
--------------------------------------- */

#define SCENARIO_JUNCTION  1
#define SCENARIO_FOLLOWING 2
#define SCENARIO_MERGING   3
#define SCENARIO_CORRIDOR  4


/* ---------------------------------------
   ROBOT DIRECTION
--------------------------------------- */

#define DIRECTION_FORWARD  1
#define DIRECTION_REVERSE  2


/* ---------------------------------------
   RESERVATION STATES
--------------------------------------- */

#define SEGMENT_FREE      0
#define SEGMENT_RESERVED  1
#define SEGMENT_OCCUPIED  2
#define SEGMENT_CLEARING  3


/* ---------------------------------------
   DECISION REASONS
--------------------------------------- */

#define REASON_CLEAR             0
#define REASON_CONFLICT          1
#define REASON_UNSAFE_DISTANCE   2
#define REASON_STALE_MESSAGE     3
#define REASON_HEARTBEAT_TIMEOUT 4
#define REASON_INVALID_REQUEST   5


/* ---------------------------------------
   ROBOT MESSAGE STRUCTURE

   Shared between Rover and Manager.
--------------------------------------- */

typedef struct
{
    uint16_t type;

    int robot_id;

    /* Position information */
    float source_x;
    float source_y;

    float current_x;
    float current_y;

    float destination_x;
    float destination_y;

    /* Movement information */
    float speed;
    float sensor_distance;
    float acceleration_magnitude;

    /* Coordination information */
    int priority;
    int status;

    int scenario;
    int segment_id;
    int direction;

    /* Segment occupancy information */
    int segment_occupied;
    int segment_clear;

    /* Heartbeat and freshness information */
    uint64_t timestamp_ms;

    uint32_t sequence;
    uint32_t heartbeat_count;

} robot_message;


/* ---------------------------------------
   MANAGER REPLY STRUCTURE

   Shared between Manager and Rover.
--------------------------------------- */

typedef struct
{
    int command;
    int reason;

    int reservation_granted;
    int segment_id;

} robot_reply;


#endif
