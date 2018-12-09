#include <iostream>
#include <vector>
#include <ros/ros.h>
class Rule {
	public:
		Rule(void) {};
		virtual ~Rule(void) {};
		virtual bool configure(void) = 0;
		virtual bool Apply(int& output) = 0;
};

class Rule1 : public Rule {
	public:
		Rule1(int* input) { 
			this->data = input; 
			this->name = "rule1";
		};

		bool configure(void) {
			bool retcod;
			this->param1 = 0.0f;
			if(ros::param::get("~"+this->name+"/first_param", this->param1)) {
				std::cout<<"first param: " << this->param1 << std::endl;
				retcod = true;
			} else {
				std::cout<<"cannot get first param"<<std::endl;
				retcod = false;
			}
		};

		bool Apply(int& output) {
			output = 11;
			std::cout<<"[Rule1-Apply] - input: "<<(*this->data)<<" | output: "<<output<<std::endl;
			return true;
		};

	private:
		int* data;
		float param1;
		std::string name;
};

class Rule2 : public Rule {
	public:
		Rule2(float* input) { 
			this->data = input; 
		};
		bool configure(void) {};
		bool Apply(int& output) {
			output = 22;
			std::cout<<"[Rule2-Apply] - input: "<<(*this->data)<<" | output: "<<output<<std::endl;
			return true;
		};

	private:
		float* data;
};


int main(int argc, char** argv) {


	int input1   = 0;
	float input2 = 0.1;
	int output;
	
	// ros initialization
	ros::init(argc, argv, "rules_node");

	std::vector<Rule*> vrules;
	Rule1* rule1 = new Rule1(&input1);
	Rule2* rule2 = new Rule2(&input2);

	vrules.push_back(rule1);
	vrules.push_back(rule2);

	for (auto it = vrules.begin(); it != vrules.end(); ++it) 
		(*it)->configure();
	
	for (auto it = vrules.begin(); it != vrules.end(); ++it) 
		(*it)->Apply(output);


	for (auto it = vrules.begin(); it != vrules.end(); ++it) 
		delete (*it);

	return 0;
}
