# makefile
#  - This is a makefile to build the files needed for the auto-park program.

# Compiler to use
CC=g++

# Flags
CFLAGS=-c -Wall

auto-park: main.o
	$(CC) main.o -o auto-park

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

run: auto-park
	./auto-park

clean:
	rm -rf *o auto-park

# EOF #