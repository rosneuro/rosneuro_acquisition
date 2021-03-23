#include <string>
#include <regex>
#include <iostream>


int main(int argc, char** argv) {

	std::string expr(argv[1]);

	printf("Expression is: %s\n", expr.c_str());

	try {
		std::regex pipe("(?<=|)(.*?)(?=|)");

		for(auto i = std::sregex_iterator(expr.begin(), expr.end(), pipe); i != std::sregex_iterator(); ++i) {
			std::smatch m = *i;
			printf("%s\n", m.str().c_str());
		}
	} catch (std::regex_error& e) {
		std::cout<<"error"<<e.what()<<std::endl;
	}



}
