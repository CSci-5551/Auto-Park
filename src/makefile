# makefile
#  - This is a makefile to build the files needed for the auto-park program.
CC=g++

# Flags
CFLAGS=-c -Wall
ARIA_INCLUDE=-I/usr/local/Aria/include
ARIA_LINK=-L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt

autoPark: autoPark.o
	$(CC) $(ARIA_INCLUDE) autoPark.o -o autoPark $(ARIA_LINK)

autoPark.o: autoPark.cpp
	$(CC) $(CFLAGS) $(ARIA_INCLUDE) -c autoPark.cpp $(ARIA_LINK)

run: autoPark
	./autoPark

clean:
	rm -rf *o autoPark logfile.txt

# EOF #
