#include "rosneuro_acquisition/EGDDevice.hpp"

#include <unistd.h>
using namespace rosneuro::acquisition;

int main(int argc, char** argv) {



	EGDDevice egddev;


	if(egddev.Open("/home/ltonin/Desktop/test.bdf") == false)
		return -1;
		
	sleep(1);
	
	if(egddev.Setup(16) == false)
		return -1;


	egddev.Dump();

	return 0;
}
