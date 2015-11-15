#ifndef ESTIMATORSFACTORY_H_
#define ESTIMATORSFACTORY_H_

#include <string>
#include <map>

#include "EstimatorsCreatorImpl.h"

class IEstimator;
class EstimatorsCreator;

class EstimatorsFactory
{
public:
	static IEstimator* create(const std::string& classname);
	static void mapCreatorInstanceToClass(const std::string& classname, EstimatorsCreator* creator);
private:
	static std::map<std::string, EstimatorsCreator*>& get_table();
};

#define REGISTER(classname) \
	private: \
	static const EstimatorsCreatorImpl<classname> creator;

#define REGISTERIMPL(classname) \
	const EstimatorsCreatorImpl<classname> classname::creator(#classname);

#endif

