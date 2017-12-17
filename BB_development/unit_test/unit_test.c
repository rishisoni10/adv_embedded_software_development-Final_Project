/******************************************************
*   File: unit_test.c
*
​* ​ ​ The MIT License (MIT)
*	Copyright (C) 2017 by Snehal Sanghvi and Rishi Soni
*   Redistribution,​ ​ modification​ ​ or​ ​ use​ ​ of​ ​ this​ ​ software​ ​ in​ ​ source​ ​ or​ ​ binary
​* ​ ​ forms​ ​ is​ ​ permitted​ ​ as​ ​ long​ ​ as​ ​ the​ ​ files​ ​ maintain​ ​ this​ ​ copyright.​ ​ Users​ ​ are
​* ​ ​ permitted​ ​ to​ ​ modify​ ​ this​ ​ and​ ​ use​ ​ it​ ​ to​ ​ learn​ ​ about​ ​ the​ ​ field​ ​ of​ ​ embedded
​* ​ ​ software.​ ​ The authors​ and​ ​ the​ ​ University​ ​ of​ ​ Colorado​ ​ are​ ​ not​ ​ liable​ ​ for
​* ​ ​ any​ ​ misuse​ ​ of​ ​ this​ ​ material.
*
*
*   Authors: Snehal Sanghvi and Rishi Soni
*   Date Edited: 5 Nov 2017
*
*   Description: Source file for unit tests
*
*
********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/stat.h> 
#include "messaging.h"
#include <mqueue.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>


#define MSG_QUEUE_UNIT_TEST1 "/unit_test_queue1"
#define UNIT_PROJ2

int total_passes = 0;
int total_fails = 0;
int total_temp_passes = 0;
int total_temp_fails = 0;
int total_lux_passes = 0;
int total_lux_fails = 0;

uint16_t config_reg;

//adding to full message queue
int add_to_full_message_queue(void)
{
	static message msg_t;
	struct mq_attr mq_attr_log;
	static mqd_t mqd_temp;

	//initializing message queue attributes
	mq_attr_log.mq_maxmsg = 10;
	mq_attr_log.mq_msgsize = sizeof(message);
	mq_attr_log.mq_flags = 0;	

	mq_unlink(MSG_QUEUE_UNIT_TEST1);
	mqd_temp = mq_open(MSG_QUEUE_UNIT_TEST1, \
						O_CREAT|O_RDWR|O_NONBLOCK, \
						0666, \
						&mq_attr_log);

	int loop_count = 0;
	int i = 0;

	while(1){
		msg_t.source_task = main_thread;
		msg_t.type = LOG_MESSAGE;
		msg_t.data = &loop_count;
		msg_t.request_type = NOT_REQUEST;
		msg_t.log_level = LOG_INFO_DATA;
		gettimeofday(&msg_t.t, NULL);	

		//should not add to full queue - check for that
		if(mq_send(mqd_temp, (const char *)&msg_t, sizeof(msg_t), 1) && i==10){
			mq_close(mqd_temp);
			total_passes++;
			return 1;
		}
		else if(i>10){
			total_fails++;
			return 0;
		}
		else{
			loop_count++;
		}	

		++i;
	}
}


//remove from empty queue
int remove_from_empty_queue(void)
{
	static message msg_t;
	struct mq_attr mq_attr_log;
	static mqd_t mqd_temp;

	//initializing message queue attributes
	mq_attr_log.mq_maxmsg = 10;
	mq_attr_log.mq_msgsize = sizeof(message);
	mq_attr_log.mq_flags = 0;	

	mq_unlink(MSG_QUEUE_UNIT_TEST1);
	mqd_temp = mq_open(MSG_QUEUE_UNIT_TEST1, \
						O_CREAT|O_RDWR|O_NONBLOCK, \
						0666, \
						&mq_attr_log);

	int loop_count = 0;

	msg_t.source_task = main_thread;
	msg_t.type = LOG_MESSAGE;
	msg_t.data = &loop_count;
	msg_t.request_type = NOT_REQUEST;
	msg_t.log_level = LOG_INFO_DATA;
	gettimeofday(&msg_t.t, NULL);	


	if(mq_receive(mqd_temp, (char *)&msg_t, sizeof(msg_t), NULL) < 0)
	{
		mq_close(mqd_temp);
		total_passes++;
		return 1;
	}
	else
	{
		total_fails++;
		mq_close(mqd_temp);
		return 0;
	}	

}

/*
//Generate mock temperature values
void generate_temp(void)
{
	int flag;
	float temperature = -40.0;		//-40 deg C: minimum temperature measureable.
	while(temperature != 130.0)		//125 deg C: maximum temperature measureable. 
	{
		flag = temp_warning_test(temperature);
		if(flag == 1)
		{
			printf("High temperature sensing - TEST SUCCESS: %f\n", temperature);
			total_temp_passes++;
		}
		else if(flag == 2)
		{
			printf("Low temperature sensing - TEST SUCCESS: %f\n", temperature);
			total_temp_passes++;
		}
		else if(flag == 3)
		{
			printf("Temperature is in SAFE range: %f\n", temperature);
			total_temp_passes++;
		}
		else
		{
			printf("Temperature out of supported range: %f\n", temperature);
			total_temp_fails++;
		}

		temperature = temperature + 5.0;
	}
}

//light sensor task 
int light_warning_test(float lux)
{
	int flag = 0;
	if(lux > DAY_THRESH)
	{
		// printf("TOO BRIGHT!!!\n");
		flag = 1;
	}
	if(lux < NIGHT_THRESH)
	{
		// printf("TOO DARK!!!\n");
		flag = 2;
	}
	if((lux == DAY_THRESH || lux == NIGHT_THRESH) || (lux > NIGHT_THRESH && lux < DAY_THRESH))
	{
		flag = 3;
	}
	return flag;
}

//Generate mock lux values
void generate_lux(void)
{
	int flag;
	float lux = 0.0;		//0.0 lux: minimum light intensity measureable.
	printf("Initial temperature is %f\n", temperature);
	while(lux != 160.0)		//125 deg C: maximum temperature measureable. 
	{
		flag = light_warning_test(lux);
		if(flag == 1)
		{
			printf("High light sensing - TEST SUCCESS: %f\n", lux);
			total_lux_passes++;
		}
		else if(flag == 2)
		{
			printf("Low light sensing - TEST SUCCESS: %f\n", lux);
			total_lux_passes++;
		}
		else if(flag == 3)
		{
			printf("Light intensity is in SAFE range: %f\n", lux);
			total_lux_passes++;
		}
		else
		{
			printf("Light intensity out of supported range: %f\n", lux);
			total_lux_fails++;
		}
		lux = lux + 0.5;
	}
}
*/

int main(){

	printf("Unit test results -\n");
	printf("---------------------------\n");

	//Test1: Adding to full message queue
	if(add_to_full_message_queue() == 1)
		printf("Added to full message queue - TEST PASS\n");
	else
		printf("Added to full message queue - TEST FAIL\n");

	//Test2: Removing from empty message queue
	if(remove_from_empty_queue() == 1)
		printf("Removed from empty message queue - TEST PASS\n");
	else
		printf("Removed from empty message queue - TEST FAIL\n");


	printf("---------------------------\n");
	printf("Test summary -\n");
	printf("MESSAGE QUEUE PASSES: %d FAILS: %d\n", total_passes, total_fails);
	printf("---------------------------\n");

}
