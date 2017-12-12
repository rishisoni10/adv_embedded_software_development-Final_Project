include sources.mk
CC = gcc
UNIT_CC = gcc

CFLAGS = -g -Wall -O0 -I $(HDRPATH)

.PHONY: build unit 

build : $(SOURCE)
	$(CC) $^ $(CFLAGS) -o main -pthread -lrt -lm

unit : unit_test.c  led.c
	$(UNIT_CC) $^ $(CFLAGS) -o unit_test -pthread -lrt -lm

clean : 
	rm -rf main unit_test log.txt
