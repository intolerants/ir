EXEC	= projeto1_3
CXX	= gcc
MAIN	= projeto1_3.c
OPTIONS	= -O3 -Wall
LIBS	= -lufrn_lynx -lm

main:	clean
	$(CXX) $(OPTIONS) $(MAIN) -I./include/ -L./lib/ $(LIBS) -o $(EXEC)

clean:
	clear
	rm -f include/*~
	rm -f lib/*~
	rm -f *~

