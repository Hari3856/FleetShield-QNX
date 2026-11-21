#ifndef RECOVERY_MANAGER_H
#define RECOVERY_MANAGER_H

#include <stdint.h>
#include <sys/types.h>
#include "fleet_common.h"

#define RM_MAX_TASKS 16
#define RM_MAX_SEGMENTS 16

typedef enum {
    RM_TASK_UNASSIGNED = 0,
    RM_TASK_ASSIGNED,
    RM_TASK_IN_PROGRESS,
    RM_TASK_RECOVERY_PENDING,
    RM_TASK_COMPLETE
} rm_task_state;

typedef enum {
    RM_ROBOT_UNKNOWN = 0,
    RM_ROBOT_AVAILABLE,
    RM_ROBOT_BUSY,
    RM_ROBOT_FAILED,
    RM_ROBOT_RESPAWNING
} rm_robot_state;

typedef struct {
    int task_id;
    int owner_robot_id;       /* 0 = unassigned */
    unsigned int required_capability; /* application-defined bitmask */
    int priority;
    rm_task_state state;
} rm_task;

typedef struct {
    int segment_id;
    int owner_robot_id;
    int state;                /* fleet_common.h SEGMENT_* value */
    int quarantined;
} rm_segment;

typedef struct {
    int robot_id;
    pid_t pid;
    rm_robot_state state;
    unsigned int capabilities;
} rm_robot;

typedef struct {
    const char *rover_executable;
    int terminate_timeout_ms;
} rm_config;

int recovery_manager_init(const rm_config *config,
                          pid_t robot1_pid, pid_t robot2_pid);
int recovery_register_task(const rm_task *task);
int recovery_set_robot_capabilities(int robot_id, unsigned int capabilities);

/* Call once when Fleet Manager declares a heartbeat/process failure. */
int recovery_handle_failure(int failed_robot_id, int segment_id,
                            int failed_task_id);

/* Only call after independent proof that the physical segment is clear. */
int recovery_confirm_segment_clear(int segment_id);

int recovery_mark_task_complete(int task_id);
int recovery_assign_task(int task_id, int robot_id);
int recovery_get_robot(int robot_id, rm_robot *out);
int recovery_get_task(int task_id, rm_task *out);
int recovery_get_segment(int segment_id, rm_segment *out);
void recovery_manager_shutdown(void);

#endif
