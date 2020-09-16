#include "MyMath.h"

CMath::CMath()
{
}

CMath::~CMath()
{
}

void CMath::limitValue(simMathReal minValue,simMathReal maxValue,simMathReal &value)
{
    if (value>maxValue)
        value=maxValue;
    if (value<minValue) 
        value=minValue;
}

void CMath::limitValue(int minValue,int maxValue,int &value)
{
    if (value>maxValue) 
        value=maxValue;
    if (value<minValue) 
        value=minValue;
}

simMathReal CMath::robustAsin(simMathReal v)
{
    if (!isRealNumberOk(v))
    {
        // GENERATE AN ERROR MESSAGE HERE: IDSNOTR_NO_NUMBER_ERROR1
        return(0.0);
    }
    if (v>=1.0)
        return(piValD2_f);
    if (v<=-1.0)
        return(-piValD2_f);
    return(asinf(v));
}

simMathReal CMath::robustAcos(simMathReal v)
{
    if (!isRealNumberOk(v))
    {
        // GENERATE AN ERROR MESSAGE HERE: IDSNOTR_NO_NUMBER_ERROR2
        return(0.0);
    }
    if (v>=1.0)
        return(0.0);
    if (v<=-1.0)
        return(piValue_f);
    return(acosf(v));
}

simMathReal CMath::robustFmod(simMathReal v,simMathReal w)
{
    if ( (!isRealNumberOk(v))||(!isRealNumberOk(w)) )
    {
        // GENERATE AN ERROR MESSAGE HERE: IDSNOTR_NO_NUMBER_ERROR3
        return(0.0);
    }
    if (w==0.0)
        return(0.0);
    return(fmod(v,w));
}

double CMath::robustmod(double v,double w)
{
    if ( (!isDoubleNumberOk(v))||(!isDoubleNumberOk(w)) )
    {
        // GENERATE AN ERROR MESSAGE HERE: IDSNOTR_NO_NUMBER_ERROR4
        return(0.0);
    }
    if (w==0.0)
        return(0.0);
    return(fmod(v,w));
}

bool CMath::isRealNumberOk(simMathReal v)
{
#ifdef SIM_MATH_DOUBLE
    return(isDoubleNumberOk(v));
#else
    return(isFloatNumberOk(v));
#endif
}

bool CMath::isFloatNumberOk(float v)
{
    return ( (!SIM_IS_NAN(v))&&(fabs(v)!=std::numeric_limits<float>::infinity()) );    
}

bool CMath::isDoubleNumberOk(double v)
{
    return ( (!SIM_IS_NAN(v))&&(fabs(v)!=std::numeric_limits<double>::infinity()) );   
}
