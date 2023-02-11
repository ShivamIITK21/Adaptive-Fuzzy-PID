#ifndef FUZZYPID_H
#define FUZZYPID_H

#include "controllers.hpp"
#include "fl/Headers.h"
#include<utility>
using namespace fl;
typedef struct gainRange{
    std::pair<double, double> deledot;
    std::pair<double, double> dele;
    std::pair<double, double> delkp;
    std::pair<double, double> delki;
    std::pair<double, double> delkd;
}gainRange;


class FuzzyPID: public Controller{
    public:
        double kp, ki, kd, Ts;
        double integral;
        double derivative;
        double old_ef;
        gainRange ranges;
        fl::Engine * engine;

        fl::InputVariable * e;
        fl::InputVariable * edot;
        fl::OutputVariable * dkp;
        fl::OutputVariable * dki;
        fl::OutputVariable * dkd;

        FuzzyPID(double _kp, double _ki, double _kd, double _Ts, gainRange _ranges);

        double update(double ref, double pos);
        void updateGains();
        void setMembershipFuncsInp(InputVariable * var, double start, double end);
        void setMembershipFuncsOut(OutputVariable * var, double start, double end);
};


#endif