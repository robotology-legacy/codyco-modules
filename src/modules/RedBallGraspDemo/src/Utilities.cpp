#include "Utilities.h"

#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

namespace codyco {

    bool Predictor::configure(Property &options)
    {
        if (net.configure(options))
        {
            net.printStructure();
            return true;
        }
        else
            return false;
    }

    Vector Predictor::predict(const Vector &head, Bottle *imdLeft, Bottle *imdRight)
    {
        Bottle *firstBlobLeft=imdLeft->get(0).asList();
        Bottle *firstBlobRight=imdRight->get(0).asList();

        Vector in(7);
        in[0]=head[3];                              // tilt
        in[1]=head[4];                              // pan
        in[2]=head[5];                              // ver
        in[3]=firstBlobLeft->get(0).asDouble();     // ul
        in[4]=firstBlobLeft->get(1).asDouble();     // vl
        in[5]=firstBlobRight->get(0).asDouble();    // ur
        in[6]=firstBlobRight->get(1).asDouble();    // vr

        return net.predict(in);
    }

    void myReport::report(const SearchReport& report, const char *context)
    {
        std::string ctx=context;
        std::string key=report.key.c_str();
        std::string prefix="";

        prefix=ctx;
        prefix+=".";

        key=prefix+key;
        if (key.substr(0,1)==".")
            key = key.substr(1,key.length());

        if (!present.check(key.c_str()))
        {
            present.put(key.c_str(),"present");
            order.addString(key.c_str());
        }

        if (report.isFound)
            actual.put(key.c_str(),report.value);

        if (report.isComment==true)
        {
            comment.put(key.c_str(),report.value);
            return;
        }

        if (report.isDefault==true)
        {
            fallback.put(key.c_str(),report.value);
            return;
        }

        if (comment.check(key.c_str()))
        {
            if (!reported.check(key.c_str()))
            {
                if (report.isFound)
                {
                    std::string hasValue=report.value.c_str();
                    if (hasValue.length()>35)
                        hasValue=hasValue.substr(0,30)+" ...";

                    fprintf(stdout,"Checking \"%s\": = %s (%s)\n",key.c_str(),
                            hasValue.c_str(),comment.check(key.c_str(),Value("")).toString().c_str());
                }
                else
                {
                    reported.put(key.c_str(),1);
                    bool hasDefault=fallback.check(key.c_str());
                    std::string defString="";

                    if (hasDefault)
                    {
                        defString+=" ";
                        defString+="(default ";
                        std::string theDefault=fallback.find(key.c_str()).toString().c_str();

                        if (theDefault=="")
                            defString+="is blank";
                        else
                            defString+=theDefault;

                        defString+=")";
                    }

                    fprintf(stdout,"Checking \"%s\": %s%s\n",key.c_str(),
                            comment.check(key.c_str(),Value("")).toString().c_str(),defString.c_str());
                }
            }
        }
    }
}
