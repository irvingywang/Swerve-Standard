#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "dji_motor.h"
#include "motor.h"
#include "pid.h"
#include "swerve_locomotion.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DJI_Motor_Handle_t *g_azimuth_motors[NUMBER_OF_MODULES];
DJI_Motor_Handle_t *g_drive_motors[NUMBER_OF_MODULES];
swerve_constants_t g_swerve_constants;
swerve_chassis_state_t g_chassis_state;

float chassis_rad;

void Chassis_Task_Init()
{
    // init common PID configuration for azimuth motors
    Motor_Config_t azimuth_motor_config = {
        .control_mode = POSITION_VELOCITY_SERIES,
        .angle_pid =
            {
                .kp = 200.0f,
                .kd = 50.0f,
                .output_limit = 100.0f,
            },
        .velocity_pid =
            {
                .kp = 200.0f,
                .ki = 0.0f,
                .kf = 0.0f,
                .feedforward_limit = 5000.0f,
                .integral_limit = 5000.0f,
                .output_limit = GM6020_MAX_CURRENT,
            }};

    // init common PID configuration for drive motors
    Motor_Config_t drive_motor_config = {
        .control_mode = VELOCITY_CONTROL,
        .velocity_pid = {
            .kp = 500.0f,
            .kd = 200.0f,
            .kf = 100.0f,
            .output_limit = M3508_MAX_CURRENT,
            .integral_limit = 3000.0f,
        }};

    // Initialize the swerve modules
    typedef struct
    {
        float azimuth_can_bus;
        float azimuth_speed_controller_id;
        float azimuth_offset;
        Motor_Reversal_t azimuth_motor_reversal;

        float drive_can_bus;
        float drive_speed_controller_id;
        Motor_Reversal_t drive_motor_reversal;
    } swerve_module_config_t;

    swerve_module_config_t module_configs[NUMBER_OF_MODULES] = {
        {2, 1, 2050, MOTOR_REVERSAL_REVERSED, 1, 1, MOTOR_REVERSAL_NORMAL},
        {2, 2, 1940, MOTOR_REVERSAL_REVERSED, 2, 2, MOTOR_REVERSAL_NORMAL},
        {2, 3, 1430, MOTOR_REVERSAL_REVERSED, 2, 3, MOTOR_REVERSAL_REVERSED},
        {2, 4, 8150, MOTOR_REVERSAL_REVERSED, 2, 4, MOTOR_REVERSAL_REVERSED}};

    // Initialize the swerve modules
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        // configure azimuth motor
        azimuth_motor_config.can_bus = module_configs[i].azimuth_can_bus;
        azimuth_motor_config.offset = module_configs[i].azimuth_offset;
        azimuth_motor_config.speed_controller_id = module_configs[i].azimuth_speed_controller_id;
        azimuth_motor_config.motor_reversal = module_configs[i].azimuth_motor_reversal;
        g_azimuth_motors[i] = DJI_Motor_Init(&azimuth_motor_config, GM6020);

        // configure drive motor
        drive_motor_config.can_bus = module_configs[i].drive_can_bus;
        drive_motor_config.speed_controller_id = module_configs[i].drive_speed_controller_id;
        drive_motor_config.motor_reversal = module_configs[i].drive_motor_reversal;
        g_drive_motors[i] = DJI_Motor_Init(&drive_motor_config, M3508);
    }

    // Initialize the swerve locomotion constants
    g_swerve_constants = swerve_init(TRACK_WIDTH, WHEEL_BASE, WHEEL_DIAMETER, SWERVE_MAX_SPEED, SWERVE_MAX_ANGLUAR_SPEED);
}

void Chassis_Ctrl_Loop()
{
    // Control loop for the chassis
}