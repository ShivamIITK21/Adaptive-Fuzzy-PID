#ifndef FUZZYPID_H
#define FUZZYPID_H

#include "controllers.hpp"
#include "fl/Headers.h"
#include<utility>

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

        FuzzyPID(std::string param, double _kp, double _ki, double _kd, double _Ts, gainRange _ranges);

        double update(double ref, double pos);
        void updateGains();
};


#endif