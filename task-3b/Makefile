.PHONY: run, analyze, valgrind, clean

server: *.cpp
	g++ -std=c++17 -Wall -Werror -g *.cpp -o server

run: server
	./server

analyze: *.cpp
	cppcheck --suppress=missingIncludeSystem --enable=all .

valgrind: server
	valgrind --leak-check=full --show-leak-kinds=all --show-error-list=yes ./server

clean:
	rm server

test:
	./test_framework.py
