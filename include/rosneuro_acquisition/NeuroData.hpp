#ifndef ROSNEURO_NEURO_DATA_HPP
#define ROSNEURO_NEURO_DATA_HPP

#include <string>
#include <vector>

namespace rosneuro {

struct NeuroDataInfo {
	std::string					name;
	std::string					unit;
	std::string					transducter;
	std::string					prefiltering;
	double						minmax[2];
	int							isint;
	unsigned int				nchannels;
	std::vector<std::string>	labels;
};

class NeuroData {
	public:
		NeuroData(unsigned int ngroups);
		virtual ~NeuroData(void);

	public:
		std::vector<void*>			data;
		std::vector<NeuroDataInfo>	info;
};

}


#endif
