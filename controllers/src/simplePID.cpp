#include "simplePID.hpp"

PID::PID(double _kp, double _ki, double _kd, double _fc, double _Ts){
    kp = _kp;
    ki = _ki;
    kd = _kd;
    alpha = calcAlphaEMA(_fc * _Ts);
    Ts = _Ts;
    integral = 0;
    old_ef = 0;
}


double PID::calcAlphaEMA(double fn){
    if (fn <= 0)
        return 1;
    const double c = std::cos(2 * float(M_PI) * fn);
    return c - 1 + std::sqrt(c * c - 4 * c + 3);
}

double PID::update(double ref, double pos){
    double error = ref - pos;
    double ef = alpha*error + (1 - alpha)*old_ef;
    double derivative = (ef - old_ef)/Ts;
    double new_integral = integral + error*Ts;

    double updated = kp*error + ki*integral + kd*derivative;

    integral = new_integral;
    old_ef = ef;

    return updated;
}