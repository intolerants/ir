EXEC	= main
CXX	= gcc
MAIN	= meta1.c
OPTIONS	= -O3 -Wall
LIBS	= -lufrn_lynx -lm

main:	clean
	$(CXX) $(OPTIONS) $(MAIN) -I./include/ -L./lib/ $(LIBS) -o $(EXEC)

clean:
	clear
	rm -f include/*~
	rm -f lib/*~
	rm -f *~

