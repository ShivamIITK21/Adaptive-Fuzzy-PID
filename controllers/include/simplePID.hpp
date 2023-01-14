#ifndef PID_H
#define PID_H

#include<cmath>
#include "controllers.hpp"

class PID: public Controller{

    private:
        double kp, ki, kd, alpha, Ts;
        double max_out;
        double integral;
        double old_ef;

    public:
        PID(double _kp, double _ki, double _kd, double _fc, double _Ts);
        static double calcAlphaEMA(double fn);
        double update(double ref, double pos);
};

#endif