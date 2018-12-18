#ifndef ROSNEURO_NEURO_DATA_CPP
#define ROSNEURO_NEURO_DATA_CPP

#include "rosneuro_acquisition/NeuroData.hpp"

namespace rosneuro {

NeuroData::NeuroData(unsigned int ngroups) {
	this->data.reserve(ngroups);
	this->info.reserve(ngroups);

	this->data.resize(ngroups, nullptr);
	this->info.resize(ngroups);
}

NeuroData::~NeuroData(void) {
	
	for(auto it = this->data.begin(); it != this->data.end(); ++it) {
		if((*it) != nullptr)
			free((*it));
	}
}


}

#endif
