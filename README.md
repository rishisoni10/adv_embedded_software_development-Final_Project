Project Overview:
We have designed a fitness tracker system that records the number of steps (Pedometer - LSM6DS3) taken by user and his pulse rate (pulse rate sensor - AD8232)

A TI TIVA TM4C1294XL microcontroller acts as the client and is setup as a data acquisition unit from sensors. An I2C-interfaced pedometer and a pulse rate sensor spit out data at regular intervals. A Beaglebone Green, connected through a serial (UART) interface with the Tiva board, serially receiving data and logs it in a log file, effectively acting as a server. Both the boards have multiple threads / tasks running, with the client side having three FreeRTOS tasks, and the server side having three Linux threads. This application is based on a multi-threading model having inter-process and inter-processor / operating system communication and transfers data between heterogeneous systems using best software practices learnt in this class.

Messaging interface: 
A custom messaging API structure has been created which has various fields like log level, source ids, request types, etc.
Several enums have been designed to support the API and we have maintained the same messaging interface on both Tiva and BBG to make it as portable as possible. To make logging easier on the server side, the message packet has an enum to idenfify the source task / thread. A uniform messaging structure between both the boards for cross - platform communication. 

Intra-task Communication:
Using the POSIX message queue API for the queue operations for Beaglebone and FreeRTOS queues 
