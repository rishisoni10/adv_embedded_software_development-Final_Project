Project Overview:
We have designed a fitness tracker system that records the number of steps taken by user and his pulse rate.
There is a Texas Instrument's TM4C1294XL microcontroller that is the client side and set up as a data acquisition unit. 
It has the I2C-interfaced pedometer and an analog interface pulse rate sensor hooked up to it which spit out data at regular intervals.
There is also a Beaglebone Green that we have used which serves as a server and keeps a record of the body vitals.
Our application is based on a multi-threading model having inter-process communication and transfers data between heterogeneous systems using best software practices learnt in this class.

Messaging interface:
We developed our own custom messaging API structure which had various fields like log level, source ids, request types, etc.
Several enums have been designed to support the API and we have maintained the same messaging interface on both Tiva and BBG to make it as portable as possible.

Intra-task Communication:
Using the POSIX message queue API for the queue operations for Beaglebone and FreeRTOS queues 
