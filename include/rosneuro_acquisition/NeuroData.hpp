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

typedef std::vector<void*>::iterator NeuroDataIt;
typedef std::vector<void*>::const_iterator NeuroDataConstIt;
typedef std::vector<NeuroDataInfo>::iterator NeuroDataInfoIt;
typedef std::vector<NeuroDataInfo>::const_iterator NeuroDataInfoConstIt;

class NeuroData {
	public:
		NeuroData(void);
		virtual ~NeuroData(void);

		void SetGroups(unsigned int ngroups);


		virtual NeuroDataIt Begin(void);
		virtual NeuroDataIt End(void);
		virtual NeuroDataConstIt Begin(void) const;
		virtual NeuroDataConstIt End(void) const;
		
		virtual NeuroDataInfoIt BeginInfo(void);
		virtual NeuroDataInfoIt EndInfo(void);
		virtual NeuroDataInfoConstIt BeginInfo(void) const;
		virtual NeuroDataInfoConstIt EndInfo(void) const;
	public:
		std::vector<void*>			data;
		std::vector<NeuroDataInfo>	info;
};

}


#endif
