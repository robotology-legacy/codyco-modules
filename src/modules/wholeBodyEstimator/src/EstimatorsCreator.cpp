#include "EstimatorsCreator.h"
#include "EstimatorsFactory.h"

EstimatorsCreator::EstimatorsCreator(const std::string& classname)
{
    EstimatorsFactory::mapCreatorInstanceToClass(classname, this);
}

