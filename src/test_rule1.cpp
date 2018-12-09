#include <iostream>

class Input {
	public:
		virtual ~Input(){};
};

template <typename T>
struct SpecificInput:Input {
	SpecificInput(T d):data(d){}
	T data;
};

class Rule {
	public:
		virtual ~Rule(void) {};
		virtual bool Apply(Input* data) =0;
};

template<typename T>
class Rule1 : public Rule {
	public:
		bool Apply(Input* data) {
			SpecificInput<T> *d = dynamic_cast<SpecificInput<T> *>(data);
			std::cout<<"Rule1-Apply: "<<d->data<<std::endl;
			return true;
		};
};

int main(int argc, char** argv) {

	
	SpecificInput<int> i1(177);
	SpecificInput<float> i2(33.3);
	SpecificInput<std::string> i3("ciao");

	Rule1<int> rule1;
	Rule1<float> rule2;
	Rule1<std::string> rule3;

	rule1.Apply(&i1);
	rule2.Apply(&i2);
	rule3.Apply(&i3);

	return 0;
}
