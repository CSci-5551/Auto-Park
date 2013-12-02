# makefile
#  - This is a makefile to build the files needed for the auto-park program.

# Compiler to use
CC=g++
CFLAGS=-c -Wall

auto-park: main.o robot.o
	$(CC) main.o robot.o -o auto-park

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

robot.o: robot.cpp robot.h
	$(CC) $(CFLAGS) -c robot.cpp robot.h

run: auto-park
	./auto-park

clean:
	rm -rf *o auto-park

# EOF #