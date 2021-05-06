CC = arm-linux-gnueabihf-gcc
CFLAGS = -g -W -Wall

TARGET = main
OBJFILES = i2c_interface.o bmi160.o sensor.o
FILES = i2c_interface.c bmi160.c sensor.c
all: clean main

main: $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES)

bmi160.o:
	$(CC) $(CFLAGS) bmi160.c -o bmi160.o -c

i2c_interface.o:
	$(CC) $(CFLAGS) i2c_interface.c -o i2c_interface.o -c

sensor.o:
	$(CC) $(CFLAGS) sensor.c -o sensor.o -c

clean:
	rm -rf *.o main
