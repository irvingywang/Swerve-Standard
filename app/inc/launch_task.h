#ifndef LAUNCH_TASK_H
#define LAUNCH_TASK_H

#include "dji_motor.h"

#define NUM_SHOTS (8)
#define SHOT_ANGLE_OFFSET_RAD (2 * PI / NUM_SHOTS)
#define NUM_BURST (5)
#define FEED_TOLERANCE (5 * PI / 180) // 5 degree tolerance 
#define FEED_RATE  (60.0 / 8 * 60) // rpm
#define FREQUENCY (8 * (FEED_RATE / DJI_MAX_TICKS) * M2006_REDUCTION_RATIO)

void Launch_Task_Init(void);
void Launch_Ctrl_Loop(void);
void handleSingleFire(void);
void startFlywheel(void);
void stopFlywheel(void);
void handleFullAuto(void);
void handleBurst(void);

/**
 * @brief Rejiggle the feed motor to prevent jams
 */
void rejiggle(void);

#endif // LAUNCH_TASK_H
