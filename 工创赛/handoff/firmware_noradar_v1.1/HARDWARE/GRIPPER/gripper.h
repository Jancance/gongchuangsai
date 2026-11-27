#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdint.h>

void Gripper_Init(void);
uint8_t jiazhua(float angle_deg);
void Gripper_Stop(void);
/* True after the last command crosses a PWM update boundary. */
uint8_t Gripper_PulseApplied(void);

#endif
