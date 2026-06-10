// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h> //printf
#include <string.h> //strcmp
#include <stdlib.h> //atoi

#include <ucdr/microcdr.h>
#include <uxr/client/client.h>

#include "uxrce_pub_topics.h"
#include "uxrce_app.h"
#include "gpio.h"
#include "drv_uart.h"

#define STREAM_HISTORY  8
#define BUFFER_SIZE     1024


uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
uint8_t input_reliable_stream_buffer[BUFFER_SIZE];

static uxrObjectId datawriter_id;

static bool connected =false;

extern uxrUartTransArgs_t uart_args;

bool HelloWorld_serialize_topic(
        ucdrBuffer* writer,
        const HelloWorld* topic)
{
    (void) ucdr_serialize_uint32_t(writer, topic->index);

    (void) ucdr_serialize_string(writer, topic->message);

    return !writer->error;
}

bool HelloWorld_deserialize_topic(
        ucdrBuffer* reader,
        HelloWorld* topic)
{
    (void) ucdr_deserialize_uint32_t(reader, &topic->index);

    (void) ucdr_deserialize_string(reader, topic->message, 255);

    return !reader->error;
}

// -----------------------------------------------------------------------------
// 自定义 ROS2 std_msgs::msg::dds_::String_ 话题类型支持函数 Definition of custom ROS2 std_msgs::msg::dds_::String_ topic type support functions
bool StdString_serialize_topic(ucdrBuffer* writer, const char* msg)
{
    (void) ucdr_serialize_string(writer, msg);
    return !writer->error;
}

uint32_t StdString_size_of_topic(const char* msg, uint32_t size)
{
    uint32_t previousSize = size;

    size += (uint32_t)(ucdr_alignment(size, 4) + 4 + strlen(msg) + 1);

    return size - previousSize;
}
// -----------------------------------------------------------------------------

uint32_t HelloWorld_size_of_topic(
        const HelloWorld* topic,
        uint32_t size)
{
    uint32_t previousSize = size;
    size += (uint32_t)(ucdr_alignment(size, 4) + 4);

    size += (uint32_t)(ucdr_alignment(size, 4) + 4 + strlen(topic->message) + 1);

    return size - previousSize;
}

int Publish_HelloWorld_Init(uxrSession* session,
                            uxrStreamId reliable_out,
                            uxrStreamId reliable_in,
                            uxrObjectId participant_id)
{
    // transport already initialized on uxrce_ping_test_()
    
    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
            "<topic>"
            "<name>rt/HelloWorldTopic</name>"
            "<dataType>std_msgs::msg::dds_::String_</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(session, reliable_out, topic_id, participant_id, topic_xml,
                    UXR_REPLACE);

    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";
    uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, reliable_out, publisher_id, participant_id,
                    publisher_xml, UXR_REPLACE);

    datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml = "<dds>"
            "<data_writer>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>rt/HelloWorldTopic</name>"
            "<dataType>std_msgs::msg::dds_::String_</dataType>"
            "</topic>"
            "</data_writer>"
            "</dds>";
    uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, reliable_out, datawriter_id, publisher_id,
                    datawriter_xml, UXR_REPLACE);

    // Send create entities message and wait its status

    uint8_t status[3];
    uint16_t requests[3] = {
        topic_req,
        publisher_req,
        datawriter_req
    };
    if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
    {
        printf("Error at create entities: participant: %u \r\n ", status[0]);
        printf("Error at create entities: topic: %u \r\n ", status[1]);
        printf("Error at create entities: publisher: %u \r\n ", status[2]);
        // uxrce_blink(LED_RED_GPIO_Port, LED_RED_Pin, 5, 100);
        return 1;
    }

    connected = true;
    return 0;

    // Clean up transport and session here (not shown for brevity)

}

void Publish_HelloWorld_Loop(uxrSession* session,
                            uxrStreamId reliable_out)
{

    if (!connected)
    {
        // Loop to publish topics   
        return;
    }

    // Write topics
    const char* msg = "Hello Micro XRCE-DDS!";

    ucdrBuffer ub;
    uint32_t topic_size = StdString_size_of_topic(msg, 0);
    uxr_prepare_output_stream(session, reliable_out, datawriter_id, &ub, topic_size);
    StdString_serialize_topic(&ub, msg);

    // printf("Send topic: %s, id: %i\n", topic.message, topic.index);
    connected = uxr_run_session_time(session, 1000);
}
