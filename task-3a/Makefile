.PHONY: run, valgrind, clean

switch: *.cpp *.h
	g++ -std=c++17 -Wall -Werror -Iframework/dep/asio/include -pthread -g *.cpp framework/control.cpp -o switch

run: switch
	./switch

analyze: *.cpp *.h
	cppcheck --std=c++17 --suppress=missingIncludeSystem --suppress=unusedFunction --enable=all *.cpp

valgrind: switch
	valgrind --leak-check=full --show-leak-kinds=all --show-error-list=yes ./switch

clean:
	rm switch
