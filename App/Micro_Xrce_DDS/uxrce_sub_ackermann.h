#ifndef UXRCE_SUB_ACKERMANN_H
#define UXRCE_SUB_ACKERMANN_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <uxr/client/client.h>

typedef struct
{
    float steering_angle;
    float steering_angle_velocity;
    float speed;
    float acceleration;
    float jerk;
} AckermannDriveCmd;

int Subscribe_Ackermann_Init(uxrSession* session,
                            uxrStreamId reliable_out,
                            uxrStreamId reliable_in,
                            uxrObjectId participant_id);

void Uxrce_SubAckermann_OnTopic(ucdrBuffer* ub, AckermannDriveCmd* cmd);


bool Uxrce_SubAckermann_GetLatest(AckermannDriveCmd* out);

#endif