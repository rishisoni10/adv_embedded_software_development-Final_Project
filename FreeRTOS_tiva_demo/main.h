/*
 * main.h
 *
 *  Created on: December 6, 2017
 *      Author: Snehal Sanghvi and Rishi Soni
 */

#ifndef MAIN_H_
#define MAIN_H_

// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U

//enum for different task ids
typedef enum{
    pedometer,
    heart_rate,
    socket
}task_id_t;

//structure packet having the task id and data to be sent
typedef struct{
    int32_t data;
    task_id_t task_id;
}message_t;

#endif /* MAIN_H_ */
