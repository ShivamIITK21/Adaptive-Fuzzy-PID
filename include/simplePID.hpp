#include<cmath>

class PID{

    private:
        double kp, ki, kd, alpha, Ts;
        double max_out;
        double integral;
        double old_ref;

    public:
        PID(double _kp, double _ki, double _kd, double _fc, double _Ts);
        static double calcAlphaEMA(float fn);
        double update(double ref, double pos);
};

