/*
 * messaging.h
 *
 *  Created on: December 11, 2017
 *      Author: Snehal Sanghvi and Rishi Soni
 */

#ifndef MAIN_H_
#define MAIN_H_

#define DATA_RECV_MSG_QUEUE "/data_recv_msg_queue_descriptor"
#define DATA_RECV_MSG_QUEUE_COPY "/data_recv_msg_queue_descriptor_copy"


#define MAX_MESSAGE_LENGTH 1000
#define ERROR (-1)
#define OK (0)


/*enumerating the message type */
typedef enum message_type_t{
    SYSTEM_FAILURE_MESSAGE,
    LOG_MESSAGE,
    SYSTEM_INIT_MESSAGE,
    REQUEST_MESSAGE,
    RESPONSE_MESSAGE,
    DUMMY_MESSAGE
}message_type;

/*Enumerating different logging levels*/
typedef enum log_level_t{
    LOG_CRITICAL_FAILURE,
    LOG_SENSOR_EXTREME_DATA,
    LOG_MODULE_STARTUP,
    LOG_INFO_DATA,
    LOG_REQUEST,
    LOG_HEARTBEAT,
    LOG_LIGHT_TRANSITION
}log_l;


//enum for different task ids
typedef enum{
    pedometer,
    pulse_rate,
    serial,
    main_t,
    data_recv,
    logger,
    send_t,
}task_id;

/*Enumerating whether the request message queue has a request
receive or request send type*/
typedef enum request_type_t{
    PED_REQUEST,   //can only be sent by pulse thread
    PULSE_REQUEST,  //can only be sent by pedometer thread
    NOT_REQUEST
}request_t;


/*Enumerating the type of request sent between light and temperature threads*/
typedef enum type_rqst{
    PED_STARTUP,
    PED_DATA,
    PULSE_STARTUP,
    PULSE_DATA,
    DATA_RECV_STARTUP,
    SEND_STARTUP,
    LOGGER_STARTUP,
    OTHER
}type_of_request;


/*Struct of the message to be sent or received*/
typedef struct message_t{
    log_l log_level;
    request_t request_type;     //only useful for the request queue
    task_id source_task;
    message_type type;
    type_of_request msg_rqst_type;
    struct timeval t;   //TODO: Timestamp
    uint32_t data;
}message;


#endif