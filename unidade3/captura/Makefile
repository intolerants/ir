# compiler
CC :=g++
# include files
CFLAGS :=`pkg-config opencv --cflags` `pkg-config opencv --libs` `lm`
LDFLAGS :=
# compile all c++ files in dir
# SOURCES :=$(wildcard *.cpp *.c)
SOURCES :=meta1.cpp al5d.c circleFinder.cpp
# C++ files .cpp removed file is executable
EXECUTABLE :=$(SOURCES:.cpp=)

all:$(EXECUTABLE)

$(EXECUTABLE):$(SOURCES)
	$(CC) $< $(LDFLAGS) $(CFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)