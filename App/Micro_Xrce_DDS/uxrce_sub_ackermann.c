#include "main.h"
#include "uxrce_app.h"
#include <stdbool.h>
#include <stdint.h>
#include <uxr/client/client.h>

#include <string.h>
#include <ucdr/microcdr.h>
#include <stdio.h>

#include "uxrce_app.h"
#include "uxrce_sub_ackermann.h"
#include "uxrce_pub_topics.h"
#include "uxrce_transport_uart.h"

static AckermannDriveCmd latest_cmd;
static volatile bool has_new_cmd = false;
static uint32_t lastest_tick = 0;

static bool Uxrce_SubAckermann_Deserialize(
    ucdrBuffer* ub,
    AckermannDriveCmd* cmd)
{
    if (!ub || !cmd) {
        return false;
    }

    ucdr_deserialize_float(ub, &cmd->steering_angle);
    ucdr_deserialize_float(ub, &cmd->steering_angle_velocity);
    ucdr_deserialize_float(ub, &cmd->speed);
    ucdr_deserialize_float(ub, &cmd->acceleration);
    ucdr_deserialize_float(ub, &cmd->jerk);

    return true;
}

int Subscribe_Ackermann_Init(uxrSession* session,
                            uxrStreamId reliable_out,
                            uxrStreamId reliable_in,
                            uxrObjectId participant_id)
{
    // Implementation for initializing Ackermann subscription
     
    // Create topic
    uxrObjectId topic_id = uxr_object_id(0x02, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
            "<topic>"
            "<name>rt/ackermann/speed</name>"
            "<dataType>ackermann_msgs::msg::dds_::AckermannDrive_</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(session, reliable_out, topic_id, participant_id, topic_xml,
                    UXR_REPLACE);

    uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    const char* subscriber_xml = "";
    uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, reliable_out, subscriber_id, participant_id,
                    subscriber_xml, UXR_REPLACE);

    uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
    const char* datareader_xml = "<dds>"
            "<data_reader>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>rt/ackermann/speed</name>"
            "<dataType>ackermann_msgs::msg::dds_::AckermannDrive_</dataType>"
            "</topic>"
            "</data_reader>"
            "</dds>";
    uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, reliable_out, datareader_id, subscriber_id,
                    datareader_xml, UXR_REPLACE);

    // Send create entities message and wait its status
    uint8_t status[3];
    uint16_t requests[3] = {
        topic_req, subscriber_req, datareader_req
    };
    if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
    {
        printf("Error at create entities: topic: %i subscriber: %i datareader: %i\n", status[0],
                status[1], status[2]);
        return 1;
    }

    // Request topics
    uxrDeliveryControl delivery_control = {
        0
    };
    // 确保持续接收topic数据，直到程序结束。设置为无限制接收样本数量。
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    uxr_buffer_request_data(session, reliable_out, datareader_id, reliable_in, &delivery_control);

    // Read topics
    return 0;
}


void Uxrce_SubAckermann_OnTopic(ucdrBuffer* ub, AckermannDriveCmd* cmd)
{

    Uxrce_SubAckermann_Deserialize(ub, cmd);

    if (!ub->error) {
        latest_cmd = *cmd;
        has_new_cmd = true;
        lastest_tick = HAL_GetTick();
    }
    
}

bool Uxrce_SubAckermann_GetLatest(AckermannDriveCmd* out, uint32_t* tick)
{
    if (!has_new_cmd) {
        return false;
    }

    *out = latest_cmd;
    *tick = lastest_tick;

    return true;
}