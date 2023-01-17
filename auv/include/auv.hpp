#ifndef AUV_H
#define AUV_H

#include "controllers.hpp"
#include "simplePID.hpp"
#include<string>
#include<iostream>

typedef struct gains{
    double kp;
    double ki;
    double kd;
    double fc;
}gains;

class AUV{
    public:
        double x, y, z, phi;
        double vx = 0, vy = 0, vz = 0, vphi = 0;
        double fx = 0, fy = 0, fz = 0, fphi = 0;
        double Ts;
        Controller * controlX;
        Controller * controlY;
        Controller * controlZ;
        Controller * controlPhi;
        double refx, refy, refz, refphi;

        AUV(std::string type, double _x, double _y, double _z, double _phi, double _Ts, gains gainX, gains gainY, gains gainZ, gains gainPhi, double _refx, double _refy, double _refz, double _refphi);

        void move();
        void accelerate();
        void controlResponse();

        ~AUV();
};

#endif