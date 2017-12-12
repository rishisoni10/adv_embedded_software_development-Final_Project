/******************************************************
*   File: led.c
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
*   Date Edited: 5 Nov 2017
*
*   Description: Source file for LED interface
*
*
********************************************************/

#include <stdio.h>
#include "led.h"

char* srcPATH = "/sys/class/leds/beaglebone:green:usr1";
char* PATH = "/sys/class/leds/beaglebone:green:usr1/trigger";
char* LEDPATH = "/sys/class/leds/beaglebone:green:usr1/brightness";

void remove_trigger(void) {
        FILE* fp = NULL;
        if((fp = fopen(PATH, "r+")))
        {
                fwrite("none", 1, 4, fp);
                fclose(fp);
        }
        else
                printf("Error\n");
}


void LEDOn(void)
{
        FILE* LED = NULL;
        remove_trigger();
        if((LED = fopen(LEDPATH, "r+")))
        {
                fwrite("1", 1, 1, LED);
                fclose(LED);
        }
        else
                printf("LEDOn error\n");
}

void LEDBlink(void)
{
    FILE* LED = NULL;
    remove_trigger();

    if((LED = fopen("/sys/class/leds/beaglebone:green:usr1/trigger", "r+")))
    {
        fwrite("timer", 1, 5, LED);
        fclose(LED);
    }       

    if((LED = fopen("/sys/class/leds/beaglebone:green:usr1/trigger/delay_on", "r+")))
    {
        fwrite("50", 1, 2, LED);
        fclose(LED);
    }

    if((LED = fopen("/sys/class/leds/beaglebone:green:usr1/trigger/delay_off", "r+")))
    {
        fwrite("50", 1, 2, LED);
        fclose(LED);
    }    
}


void LEDOff(void)
{
        FILE* LED = NULL;
        remove_trigger();
        if((LED = fopen(LEDPATH, "r+")))
        {
                fwrite("0", 1, 1, LED);
                fclose(LED);
        }
        else
                printf("LEDOff error\n");
}
