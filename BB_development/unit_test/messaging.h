/******************************************************
*   File: messaging.h
*
​* ​ ​ The MIT License (MIT)
*   Copyright (C) 2017 by Snehal Sanghvi and Rishi Soni
*   Redistribution,​ ​ modification​ ​ or​ ​ use​ ​ of​ ​ this​ ​ software​ ​ in​ ​ source​ ​ or​ ​ binary
​* ​ ​ forms​ ​ is​ ​ permitted​ ​ as​ ​ long​ ​ as​ ​ the​ ​ files​ ​ maintain​ ​ this​ ​ copyright.​ ​ Users​ ​ are
​* ​ ​ permitted​ ​ to​ ​ modify​ ​ this​ ​ and​ ​ use​ ​ it​ ​ to​ ​ learn​ ​ about​ ​ the​ ​ field​ ​ of​ ​ embedded
​* ​ ​ software.​ ​ The authors​ and​ ​ the​ ​ University​ ​ of​ ​ Colorado​ ​ are​ ​ not​ ​ liable​ ​ for
​* ​ ​ any​ ​ misuse​ ​ of​ ​ this​ ​ material.
*
*
*   Authors: Snehal Sanghvi and Rishi Soni
*   Date Edited: 24 Oct 2017
*
*   Description: Header file for messaging interface
*
*
********************************************************/

#ifndef _MESSAGING_H
#define _MESSAGING_H

#include <time.h>

#define TEMP_MSG_QUEUE_RESPONSE "/temp_log_queue_response"
#define TEMP_MSG_QUEUE_RESPONSE_COPY "/temp_log_queue_response_copy"
#define LIGHT_MSG_QUEUE_RESPONSE "/light_log_queue_response"
#define LIGHT_MSG_QUEUE_RESPONSE_COPY "/light_log_queue_response_copy"
#define MSG_QUEUE_REQUEST "/queue_request"
#define MSG_QUEUE_TEMP_HB "/temp_heartbeat_queue"
#define MSG_QUEUE_LIGHT_HB "/light_heartbeat_queue"

#define MAX_MESSAGE_LENGTH 1000
#define ERROR (-1)
#define OK (0)

/*Enumerating the threads*/
typedef enum task_id_t{
    main_thread = 1,
    temp_thread = 2,
    lght_thread = 3,
    log_thread = 4,
    dec_thread = 5,
    kill_thread_ = 6
}task_id;


typedef enum{
    CONFIG_REG_READ
}temp_request_type;

/*enumerating the message type */
typedef enum message_type_t{
    SYSTEM_FAILURE_MESSAGE,
    LOG_MESSAGE,
    SYSTEM_INIT_MESSAGE,
    REQUEST_MESSAGE,
    RESPONSE_MESSAGE,
    DUMMY_MESSAGE,
    KILL_MESSAGE
}message_type;


/*Enumerating different logging levels*/
typedef enum log_level_t{
    LOG_CRITICAL_FAILURE,
    LOG_SENSOR_EXTREME_DATA,
    LOG_MODULE_STARTUP,
    LOG_INFO_DATA,
    LOG_REQUEST,
    LOG_HEARTBEAT, 
    LOG_LIGHT_TRANSITION,
    LOG_KILL
}log_l;


/*Enumerating whether the request message queue has a request
receive or request send type*/
typedef enum request_type_t{
    TEMP_REQUEST,   //can only be sent by light thread
    LIGHT_REQUEST,  //can only be sent by temp thread
    NOT_REQUEST
}request_t;


/*Enumerating the type of request sent between light and temperature threads*/
typedef enum type_rqst{
    TEMP_CONFIG_READ,
    TEMP_SHUTDOWN_ENABLE,
    TEMP_SHUTDOWN_DISABLE,
    TEMP_CHANGE_CONVERSION_RATE,
    LIGHT_POWER_ON,
    LIGHT_POWER_OFF,
    LIGHT_CONTROL_REG_READ,
    LIGHT_SET_INTEGRATION_TIME,
    LIGHT_ID_READ,
    LIGHT_INTERRUPT_ENABLE,
    LIGHT_INTERRUPT_DISABLE
}type_of_request;

/*Struct of the message to be sent or received*/
typedef struct message_t{
    log_l log_level;
    request_t request_type;     //only useful for the request queue
    task_id source_task;
    message_type type;
    type_of_request msg_rqst_type;
    struct timeval t;   //for timestamps
    void *data;
}message;


/*Enumerating the errors for checksum*/
typedef enum ERR_t {
    CHKSUM_FAIL = 1,
    CHKSUM_PASS = 2
}ERR;


/* Function to decode the incoming message packet*/
void decode_message (void);

/*Function to validate the checksum at the end of each message using addition algorithm*/
ERR checksum_validate (void);

#endif
