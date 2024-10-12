#include "launch_task.h"

#include "dji_motor.h"
#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "referee_system.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DJI_Motor_Handle_t *g_flywheel_left, *g_flywheel_right, *g_feed_motor;

void Launch_Task_Init()
{
    // Init Launch Hardware
    Motor_Config_t flywheel_left_config = {
        .can_bus = 1,
        .speed_controller_id = 4,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_REVERSED,
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = M3508_MAX_CURRENT,
            },
    };

    Motor_Config_t flywheel_right_config = {
        .can_bus = 1,
        .speed_controller_id = 5,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = M3508_MAX_CURRENT,
            },
    };

    Motor_Config_t feed_speed_config = {
        .can_bus = 1,
        .speed_controller_id = 2,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL | POSITION_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid =
            {
                .kp = 500.0f,
                .kd = 200.0f,
                .kf = 100.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
        .angle_pid =
            {
                .kp = 450000.0f,
                .kd = 15000000.0f,
                .ki = 0.1f,
                .output_limit = M2006_MAX_CURRENT,
                .integral_limit = 1000.0f,
            }
    };

    g_flywheel_left = DJI_Motor_Init(&flywheel_left_config,M3508);
    g_flywheel_right = DJI_Motor_Init(&flywheel_right_config,M3508);
    g_feed_motor = DJI_Motor_Init(&feed_speed_config,M2006);
}

void Launch_Ctrl_Loop()
{
    // Control loop for launch
    switch (g_robot_state.launch.fire_mode)
    {
    case SINGLE_FIRE:
        
        break;
    case BURST:
        // TODO: Complete 5 burst 
        break;
    case FULL_AUTO:

        break;    
    default:
        break;
    }
}
