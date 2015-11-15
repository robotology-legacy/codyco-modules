#include "EstimatorsFactory.h"
#include "EstimatorsCreator.h"

void EstimatorsFactory::mapCreatorInstanceToClass(const std::string& classname, EstimatorsCreator* creator)
{
	get_table()[classname] = creator;
}

IEstimator* EstimatorsFactory::create(const std::string& classname)
{
	std::map<std::string, EstimatorsCreator*>::iterator i;
	i = get_table().find(classname);

	if(i != get_table().end())
		return i->second->create();
	else
		return (IEstimator*)NULL;
}

std::map<std::string, EstimatorsCreator*>& EstimatorsFactory::get_table()
{
	static std::map<std::string, EstimatorsCreator*> table;
	return table;
}

