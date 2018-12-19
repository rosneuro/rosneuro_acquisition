#ifndef ROSNEURO_NEURO_DATA_CPP
#define ROSNEURO_NEURO_DATA_CPP

#include "rosneuro_acquisition/NeuroData.hpp"

namespace rosneuro {

NeuroData::NeuroData(void) {
}


NeuroData::~NeuroData(void) {
	
	for(auto it = this->data.begin(); it != this->data.end(); ++it) {
		if((*it) != nullptr)
			free((*it));
	}
}

void NeuroData::SetGroups(unsigned int ngroups) {
	this->data.reserve(ngroups);
	this->info.reserve(ngroups);

	this->data.resize(ngroups, nullptr);
	this->info.resize(ngroups);
}

NeuroDataIt NeuroData::Begin(void) {
	NeuroDataIt it = this->data.begin();
	return it;
}

NeuroDataIt NeuroData::End(void) {
	NeuroDataIt it = this->data.end();
	return it;
}

NeuroDataConstIt NeuroData::Begin(void) const {
	NeuroDataConstIt it = this->data.begin();
	return it;
}

NeuroDataConstIt NeuroData::End(void) const {
	NeuroDataConstIt it = this->data.end();
	return it;
}

NeuroDataInfoIt NeuroData::BeginInfo(void) {
	NeuroDataInfoIt it = this->info.begin();
	return it;
}

NeuroDataInfoIt NeuroData::EndInfo(void) {
	NeuroDataInfoIt it = this->info.end();
	return it;
}

NeuroDataInfoConstIt NeuroData::BeginInfo(void) const {
	NeuroDataInfoConstIt it = this->info.begin();
	return it;
}

NeuroDataInfoConstIt NeuroData::EndInfo(void) const {
	NeuroDataInfoConstIt it = this->info.end();
	return it;
}

}

#endif
