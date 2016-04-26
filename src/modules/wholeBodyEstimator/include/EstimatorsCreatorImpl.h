#ifndef ESTIMATORSCREATORIMPL_H_
#define ESTIMATORSCREATORIMPL_H_

#include <string>

#include "EstimatorsCreator.h"

template <class T>
class EstimatorsCreatorImpl : public EstimatorsCreator
{
public:
	EstimatorsCreatorImpl<T>(const std::string& classname) : EstimatorsCreator(classname) {}
	virtual ~EstimatorsCreatorImpl<T>() {}

	virtual IEstimator* create() { return new T; }
};

#endif

