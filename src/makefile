# makefile
#  - This is a makefile to build the files needed for the auto-park program.
CXX = g++
CFLAGS = -fPIC -g -Wall
ARIA_INCLUDE =-I/usr/local/Aria/include
ARIA_LINK =-L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt

autoPark: autoPark.cpp

%.cpp: %.cpp
	$(CXX) $(CFLAGS) $(ARIA_INCLUDE) $< -o $@ $(ARIA_LINK)

run: autoPark
	./autoPark

clean:
	rm autoPark

# EOF #