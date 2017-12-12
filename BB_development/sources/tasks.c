/******************************************************
*   File: tasks.c
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
*   Date Edited: 11 Dec 2017
*
*   Description: Source file for tasks
*
*
********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/stat.h> 
#include <mqueue.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include "messaging.h"
#include "led.h"

#define LOG_BUFFER_SIZE 10000

extern int fd;
extern void uart_init(void);

uint32_t read_config = 1;
int retval;

pthread_t data_recv_thread;
pthread_t logger_thread;
pthread_t send_thread;

pthread_mutex_t mutex_data_recv;
pthread_mutex_t mutex_log;
pthread_mutex_t mutex_send;


char buf[LOG_BUFFER_SIZE];
struct sigaction sig;

FILE *fp;

static volatile int loop_count = 0;
char *filename_cmd;

struct mq_attr mq_attr_log;
static mqd_t mqd_data_recv, mqd_data_recv_copy;

const char* log_levels[] = {"LOG_CRITICAL_FAILURE", "LOG_SENSOR_EXTREME_DATA", \
 "LOG_MODULE_STARTUP", "LOG_INFO_DATA", "LOG_REQUEST", "LOG_HEARTBEAT", "LOG_LIGHT_TRANSITION"};

void signal_handler(int signum)
{
	if (signum == SIGINT)
	{
		printf("\nClosing mqueue and file...\n");
		//LEDOff();
		mq_close(mqd_data_recv);
		mq_close(mqd_data_recv_copy);
		fclose(fp);
		exit(0);
	}
}


//Start function of data receive thread
void *data_recv_thread_fn(void *threadid){
	printf("In data receive function.\n");
	int str_len;
	
	char c = '0';
	char buffer[500];
	//char parse_buffer[500];

	char *str, *str1, *str2, *str3, *str4, *str5;
    char *tk, *tk1, *tk2, *tk3, *tk4, *tk5;
    char *buff1, *buff2, *buff3, *buff4, *buff5;
    char *saveptr, *saveptr1, *saveptr2, *saveptr3, *saveptr4, *saveptr5;

	static message msg_temp, msg_temp_cp;

	task_id uart_id;
	uint32_t uart_data = 0;
	log_l uart_log_level;
	message_type uart_msg_type; 
	request_t uart_request_t;
	type_of_request uart_type_of_rqst;

	while(1){

		memset(&msg_temp, 0, sizeof(msg_temp));
		/*
		UART
		receiving data from uart into uart_data, uart_log_level, uart_id, 
		uart_msg_type, uart_request_t, uart_type_of_rqst

		*/
		memset(buffer, 0, 500);
        //if(tcsetattr(fd, TCSAFLUSH, configure) < 0)
        //{
        //      perror("ERROR in set attr\n");
        //}
        tcflush(fd, TCOFLUSH);

        while(1)
        {
        	// c = poll_for_data();    
        	// append(buffer, c);
        	read(fd, &c, 1);
        	str_len = strlen(buffer);
	        buffer[str_len] = c;
	        buffer[str_len+1] = '\0';

        	if(c == '\n')
        	{
        		// strcpy(parse_buffer, buffer);
                // msg_temp = parse(parse_buffer);

                /*Parsing block starts here....*/
                buff1 = malloc(strlen(buffer)+1);
		        strcpy(buff1, buffer);

		        buff2 = malloc(strlen(buffer)+1);
		        strcpy(buff2, buffer);

		        buff3 = malloc(strlen(buffer)+1);
		        strcpy(buff3, buffer);

		        buff4 = malloc(strlen(buffer)+1);
		        strcpy(buff4, buffer);

		        buff5 = malloc(strlen(buffer)+1);
		        strcpy(buff5, buffer);

		        printf("Parsed buffer:");
		        printf("%s", buffer);

		        str = strstr(buffer, "Log_level");
		        tk = strtok_r(str, " |", &saveptr);
		        tk = strtok_r(NULL, " |", &saveptr);
		        uart_log_level = atoi(tk);
		        // printf("Log level is:%d\n",uart_log_level);

				str1 = strstr(buff1, "Request_type");
				tk1 = strtok_r(str1, " |", &saveptr1);
				tk1 = strtok_r(NULL, " |", &saveptr1);
				uart_request_t = atoi(tk1);
				// printf("Request type is:%d\n",uart_request_t);

				str2 = strstr(buff2, "Source_task");
				tk2 = strtok_r(str2, " |", &saveptr2);
				tk2 = strtok_r(NULL, " |", &saveptr2);
				uart_id = atoi(tk2);
				// printf("Source_task is:%d\n", uart_id);

		        str3 = strstr(buff3, "Message_type");
		        tk3 = strtok_r(str3, " |", &saveptr3);
		        tk3 = strtok_r(NULL, " |", &saveptr3);
		        uart_msg_type = atoi(tk3);
		        // printf("Message type is:%d\n", uart_msg_type);

		        str4 = strstr(buff4, "Msg_rqst_type");
		        tk4 = strtok_r(str4, " |", &saveptr4);
		        tk4 = strtok_r(NULL, " |", &saveptr4);
		        uart_type_of_rqst = atoi(tk4);
		        // printf("Msg_rqst_type is:%s\n", tk4);

		        str5 = strstr(buff5, "Data");
		        tk5 = strtok_r(str5, " |", &saveptr5);
		        tk5 = strtok_r(NULL, " |", &saveptr5);
		        uart_data = atoi(tk5);
		        printf("Data is:%d\n", uart_data);

		        memset(buffer, 0, 500);
		        free(buff1);
		        free(buff2);
		        free(buff3);
		        free(buff4);
		        free(buff5);
                memset(buffer, 0, 500);
                break;
        	}
        }

		//Constructing message that contains temp data and is sent to logger
		msg_temp.source_task = uart_id;
		msg_temp.type = uart_msg_type;
		msg_temp.data = uart_data;
		msg_temp.request_type = uart_request_t;
		msg_temp.log_level = uart_log_level;
		msg_temp.msg_rqst_type = uart_type_of_rqst;
		gettimeofday(&msg_temp.t, NULL);


		//printf("timestamp in secs is %ld\n", msg1.t.tv_sec);
		//printf("timestamp in usecs is %ld\n", msg1.t.tv_usec);

		if(mq_send(mqd_data_recv, (const char *)&msg_temp, sizeof(msg_temp), 1)){
			printf("Data receive thread could not send data to logger.\n");
		}


		//Constructing message that contains copy of temp data and is sent to main
		memset(&msg_temp, 0, sizeof(msg_temp));

		msg_temp_cp.source_task = uart_id;
		msg_temp_cp.type = uart_msg_type;
		msg_temp_cp.data = uart_data;
		msg_temp_cp.request_type = uart_request_t;
		msg_temp_cp.log_level = uart_log_level;
		msg_temp_cp.msg_rqst_type = uart_type_of_rqst;
		gettimeofday(&msg_temp_cp.t, NULL);

		if(mq_send(mqd_data_recv_copy, (const char *)&msg_temp_cp, sizeof(msg_temp_cp), 1)){
			printf("Data receive thread could not send data copy to send thread.\n");
		}

		usleep(600);
	}
}


//Start function of logger thread
void *logger_thread_fn(void *threadid){
	printf("In logger thread function.\n");
	static message msg_t;
        
	static int ped_count_log = 0;
	static int ped_count_init = 0;
	static int pulse_count_log = 0;
	static int pulse_count_init = 0;
	static int ped_count_rqst = 0;
	static int pulse_count_rqst = 0;

	fp = fopen(filename_cmd, "a+");

	while(1){
		//checking the mqd_data_recv queue
		if(!mq_receive(mqd_data_recv, (char *)&msg_t, sizeof(msg_t), NULL)){
			printf("Logger thread could not receive data from data receive thread.\n");
		}	
		else{
			if(msg_t.type == (LOG_MESSAGE)){
				if(msg_t.source_task == pedometer){
					ped_count_log++;
					sprintf(buf, "[%ld secs] [%s] Source: Pedometer thread; Data: %d steps\n", \
					msg_t.t.tv_sec, log_levels[msg_t.log_level], msg_t.data);
					if(ped_count_log%100 == 1){ 
						fwrite(buf, sizeof(char), strlen(buf), fp);
					}
					memset(buf, 0, LOG_BUFFER_SIZE);
				}
				else if(msg_t.source_task == pulse_rate){
					pulse_count_log++;
					sprintf(buf, "[%ld secs] [%s] Source: Pulse rate thread; Data: %d bpm\n", \
					msg_t.t.tv_sec, log_levels[msg_t.log_level], msg_t.data);
					if(pulse_count_log%100 == 1){
						fwrite(buf, sizeof(char), strlen(buf), fp);
					}
					memset(buf, 0, LOG_BUFFER_SIZE);
				}
			}
			else if(msg_t.type == (SYSTEM_INIT_MESSAGE)){
				if(msg_t.msg_rqst_type == PED_STARTUP){
					ped_count_init++;
					sprintf(buf, "[%ld secs] [%s] Pedometer task spawned on Tiva\n", \
					msg_t.t.tv_sec, log_levels[msg_t.log_level]);
					if(ped_count_init%100 == 1){
						fwrite(buf, sizeof(char), strlen(buf), fp);
					}
					memset(buf, 0, LOG_BUFFER_SIZE);
				}
				else if(msg_t.msg_rqst_type == PULSE_STARTUP){
					pulse_count_init++;
					sprintf(buf, "[%ld secs] [%s] Pulse rate task spawned on Tiva\n", \
					msg_t.t.tv_sec, log_levels[msg_t.log_level]);
					if(pulse_count_init%100 == 1){
						fwrite(buf, sizeof(char), strlen(buf), fp);
					}
					memset(buf, 0, LOG_BUFFER_SIZE);
				}
			}
			else if(msg_t.type == (REQUEST_MESSAGE)){
				if(msg_t.source_task == pedometer){
					ped_count_rqst++;
					sprintf(buf, "[%ld secs] [%s] Pedometer task received data REQUEST from Pulse rate task.\n", \
					msg_t.t.tv_sec, log_levels[msg_t.log_level]);
					if(ped_count_rqst%100 == 1){
						fwrite(buf, sizeof(char), strlen(buf), fp);
					}
					memset(buf, 0, LOG_BUFFER_SIZE);
				}
				else if(msg_t.source_task == pulse_rate){
					pulse_count_rqst++;
					sprintf(buf, "[%ld secs] [%s] Pulse rate task received data REQUEST from Pedometer task.\n", \
					msg_t.t.tv_sec, log_levels[msg_t.log_level]);
					if(pulse_count_rqst%100 == 1){
						fwrite(buf, sizeof(char), strlen(buf), fp);
					}
					memset(buf, 0, LOG_BUFFER_SIZE);
				}
			}						
		}

		usleep(900);
	}
}


//Start function of logger thread
void *send_thread_fn(void *threadid){
	static message msg_t;
	char steps_status=0, heartbeat_status=0;
	static char uart_send_buffer[100];

	while(1){
		if(mq_receive(mqd_data_recv_copy, (char *)&msg_t, sizeof(msg_t), NULL) < 0){
			
		}	
		else{
			if(msg_t.source_task == pedometer){
				if(msg_t.data > 500){
					printf("That's a lot of steps for today!\n");
					steps_status = 1;
					LEDOn();	//LED1 ON
				}
				else{
					steps_status = 0;
				}
			}
			else if(msg_t.source_task == pulse_rate){
				if(msg_t.data > 90){
					printf("Very fast beating heart!\n");
					heartbeat_status = 1;
					LEDOn();	//glow the error LED
				}
				else{
					heartbeat_status = 0;
				}
			}

			if(!steps_status && !heartbeat_status){
				LEDOff();	//No error
			}

			//TO-DO: UART send to Tiva
			memset(uart_send_buffer, 0, sizeof(uart_send_buffer));
			sprintf(uart_send_buffer, "Steps: %d, Heartbeat: %d", steps_status, heartbeat_status);
			//UART Send buffer
		}

		usleep(900);
	}
}


int main(int argc, char const *argv[]){

	pthread_attr_t attr;
	// clock_t temp_time;

	//Initializing serial interface (UART2) in uart.c 
	uart_init();

	//checking command line options
	if(argc != 2){
		printf ("USAGE: <filename.txt>\n");
		exit(1);
	}
	else{	
		filename_cmd = argv[1];
	}

   if(pthread_mutex_init(&mutex_data_recv, NULL)){
      printf("Error in initializing data receive mutex.\n");  
      exit(1);
   }

   if(pthread_mutex_init(&mutex_log, NULL)){
      printf("Error in initializing log mutex.\n");  
      exit(1);
   }

   if(pthread_mutex_init(&mutex_send, NULL)){
      printf("Error in initializing send mutex.\n");  
      exit(1);
   }

	//initializing message queue attributes
	mq_attr_log.mq_maxmsg = 10;
	mq_attr_log.mq_msgsize = sizeof(message);
	mq_attr_log.mq_flags = 0;

	mq_unlink(DATA_RECV_MSG_QUEUE);
	mq_unlink(DATA_RECV_MSG_QUEUE_COPY);

	mqd_data_recv = mq_open(DATA_RECV_MSG_QUEUE, \
						O_CREAT|O_RDWR|O_NONBLOCK, \
						0666, \
						&mq_attr_log);

	mqd_data_recv_copy = mq_open(DATA_RECV_MSG_QUEUE_COPY, \
						O_CREAT|O_RDWR|O_NONBLOCK, \
						0666, \
						&mq_attr_log);


  	signal(SIGINT, signal_handler);
	
	pthread_attr_init(&attr);

	//spawn data recive thread
	if(pthread_create(&data_recv_thread, &attr, (void*)&data_recv_thread_fn, NULL)){
        printf("Failed to create data receive thread.\n");
	}

	//spawn logger thread
	if(pthread_create(&logger_thread, &attr, (void*)&logger_thread_fn, NULL)){
        printf("Failed to create logger thread.\n");
	}
	
	/*
	//spawn send thread
	if(pthread_create(&send_thread, &attr, (void*)&send_thread_fn, NULL)){
        printf("Failed to create send thread.\n");
	}
	*/
 

	while(1)
	{

			/*
			//checking the temp queue
			if(mq_receive(mqd_temp_cp, (char *)&msg_te, sizeof(msg_te), NULL) < 0)
			{
				// printf("Main thread could not receive data from temp thread.\n");
			}	
			else
			{
				if(*(float *)msg_te.data > TEMP_UPPER - 10 && *(float *)msg_te.data < TEMP_UPPER + 10)
				{
					printf("TOO HOT!!!\n");
					LEDBlink();	//LED1 Blinking
					msg_te.log_level = LOG_SENSOR_EXTREME_DATA;
					if(mq_send(mqd_temp, (const char *)&msg_te, sizeof(msg_te), 1) < 0)
						printf("Main thread could not send temp extreme data to logger.\n");
				}
				if(*(float *)msg_te.data > TEMP_LOWER - 10 && *(float *)msg_te.data < TEMP_LOWER + 10)
				{
					printf("TOO COLD!!!\n");
					LEDBlink();	//LED1 Blinking					
					msg_te.log_level = LOG_SENSOR_EXTREME_DATA;
					if(mq_send(mqd_temp, (const char *)&msg_te, sizeof(msg_te), 1) < 0)
						printf("Main thread could not send temp extreme data to logger.\n");
					// signal_handler(SIGINT);
				}
				
			}*/

			

		usleep(1000);
	}
	
	//join threads
 	pthread_join(data_recv_thread, NULL);
 	pthread_join(logger_thread, NULL);
 	//pthread_join(send_thread, NULL);

	return 0;
}
