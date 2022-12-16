# Makefile for part1
# Compile, using gcc
CXX = g++
CXXFLAGS = -std=c++17 -Wall
LXXFLAGS = -std=c++17
OBJECTS = part1.o
TARGET = part1

all: $(TARGET)

$(TARGET): $(OBJECTS)
	 $(CXX) $(LXXFLAG) $(OBJECTS) -o $(TARGET)

part1.o: part1.cpp
	$(CXX) $(CXXFLAGS) -c part1.cpp

.PHONY: clean
clean:
	del $(TARGET) $(TARGET).exe $(OBJECTS)
