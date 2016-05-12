# # compiler
# CC :=g++
# # include files
# CFLAGS :=`pkg-config opencv --cflags` `pkg-config opencv --libs`
# LDFLAGS :=
# OPTIONS	= -O3 -Wall
# LIBS	= -lufrn_lynx -lm
# # compile all c++ files in dir
# SOURCES :=$(wildcard *.cpp)
# # C++ files .cpp removed file is executable
# EXECUTABLE :=$(SOURCES:.cpp=)

# all:$(EXECUTABLE)

# $(EXECUTABLE):$(SOURCES)
# 	$(CC) $(OPTIONS) -I./include/ -L./lib/ $(LIBS) $< $(LDFLAGS) $(CFLAGS) -o $@

# clean:
# 	clear
# 	rm -rf $(EXECUTABLE)
# 	rm -f include/*~
# 	rm -f lib/*~

EXEC	= meta3
CXX	= gcc
MAIN	= meta3.c
OPTIONS	= -O3 -Wall
LIBS	= -lufrn_lynx -lm

main:	clean
	$(CXX) $(OPTIONS) $(MAIN) -I./include/ -L./lib/ $(LIBS) -o $(EXEC)

clean:
	clear
	rm -f include/*~
	rm -f lib/*~
	rm -f *~
