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

#define STREAM_HISTORY  4
#define BUFFER_SIZE     1024

uxrSession session;

uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
uint8_t input_reliable_stream_buffer[BUFFER_SIZE];

static uxrStreamId reliable_out;
static uxrObjectId datawriter_id;
static uxrObjectId participant_id;

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

uint32_t HelloWorld_size_of_topic(
        const HelloWorld* topic,
        uint32_t size)
{
    uint32_t previousSize = size;
    size += (uint32_t)(ucdr_alignment(size, 4) + 4);

    size += (uint32_t)(ucdr_alignment(size, 4) + 4 + strlen(topic->message) + 1);

    return size - previousSize;
}

int Publish_HelloWorld_Init(uxrCustomTransport* transport, int argc, char** argv)
{
    // transport already initialized on uxrce_ping_test_()
    
    // Create Session
    uxr_init_session(&session, &transport->comm, 0xAAAABBBB);
    if (!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
        uxrce_blink(LED_RED_GPIO_Port, LED_RED_Pin, 5, 100);
        return 1;
    }

    // Streams
    reliable_out =
        uxr_create_output_reliable_stream(&session,
                                        output_reliable_stream_buffer,
                                        BUFFER_SIZE,
                                        STREAM_HISTORY);

    uxr_create_input_reliable_stream(&session,
                                    input_reliable_stream_buffer,
                                    BUFFER_SIZE,
                                    STREAM_HISTORY);

    // Create entities
    participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>default_xrce_participant</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0,
                    participant_xml, UXR_REPLACE);

    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
            "<topic>"
            "<name>HelloWorldTopic</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml,
                    UXR_REPLACE);

    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";
    uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id,
                    publisher_xml, UXR_REPLACE);

    datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml = "<dds>"
            "<data_writer>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>HelloWorldTopic</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</data_writer>"
            "</dds>";
    uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session, reliable_out, datawriter_id, publisher_id,
                    datawriter_xml, UXR_REPLACE);

    // Send create entities message and wait its status
    uint8_t status[4];
    uint16_t requests[4] = {
        participant_req, topic_req, publisher_req, datawriter_req
    };
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 4))
    {
        printf("Error at create entities: participant: %i topic: %i publisher: %i datawriter: %i\n", status[0],
                status[1], status[2], status[3]);
        return 1;
    }
    connected = true;


//     if (participant_req == UXR_INVALID_REQUEST_ID) {
//     return 2;
// }
// if (topic_req == UXR_INVALID_REQUEST_ID) {
//     return 3;
// }
// if (publisher_req == UXR_INVALID_REQUEST_ID) {
//     return 4;
// }
// if (datawriter_req == UXR_INVALID_REQUEST_ID) {
//     return 5;
// }

// /* 先只把 create entity 请求发出去，不等待 status */
// uxr_run_session_time(&session, 100);

// connected = true;
// return 0;

    // Delete resources
    // uxr_delete_session(&session);

    // Clean up transport and session here (not shown for brevity)

    // return 0;
}

void Publish_HelloWorld_Loop(void)
{

    if (!connected)
    {
        // Loop to publish topics   
        return;
    }

    static uint32_t count = 0;

    // Write topics
    HelloWorld topic = {
            ++count, "Hello DDS world!"
        };

        ucdrBuffer ub;
        uint32_t topic_size = HelloWorld_size_of_topic(&topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, datawriter_id, &ub, topic_size);
        HelloWorld_serialize_topic(&ub, &topic);

        // printf("Send topic: %s, id: %i\n", topic.message, topic.index);
        connected = uxr_run_session_time(&session, 1000);
}