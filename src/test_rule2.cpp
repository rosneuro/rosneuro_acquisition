#include <iostream>
#include <vector>

template <typename T>
class Rule {
	public:
		virtual ~Rule(void) {};
		virtual bool Apply(const T& input, int& output) =0;
};

class Rule1 : public Rule<int> {
	public:
		bool Apply(const int& input, int& output) {
			output = 11;
			std::cout<<"[Rule1-Apply] - output: "<<output<<std::endl;
			return true;
		};
};

class Rule2 : public Rule<int> {
	public:
		bool Apply(const int& input, int& output) {
			output = 22;
			std::cout<<"[Rule2-Apply] - output: "<<output<<std::endl;
			return true;
		};
};

int main(int argc, char** argv) {


	int input1, output1;
	std::vector<Rule<int>*> vrules;

	Rule1* rule1 = new Rule1;
	Rule2* rule2 = new Rule2;

	vrules.push_back(rule1);
	vrules.push_back(rule2);


	//Rule<int>* prule = static_cast<Rule<int>*>(rule1);
	
	for (auto it = vrules.begin(); it != vrules.end(); ++it) 
		(*it)->Apply(input1, output1);


	for (auto it = vrules.begin(); it != vrules.end(); ++it) 
		delete (*it);
	return 0;
}
