#include "auv.hpp"

AUV::AUV(std::string type, double _x, double _y, double _z, double _phi, double _Ts, gains gainX, gains gainY, gains gainZ, gains gainPhi, double _refx, double _refy, double _refz, double _refphi){
    if(type == "PID"){
        x = _x;
        y = _y;
        z = _z;
        phi = _phi;
        Ts = _Ts;

        refx = _refx;
        refy = _refy;
        refz = _refz;
        refphi = _refphi;

        PID * _controlX = new PID(gainX.kp, gainX.ki, gainX.kd, gainX.fc, _Ts);
        PID * _controlY = new PID(gainY.kp, gainY.ki, gainY.kd, gainY.fc, _Ts);  
        PID * _controlZ = new PID(gainZ.kp, gainZ.ki, gainZ.kd, gainZ.fc, _Ts);
        PID * _controlPhi = new PID(gainPhi.kp, gainPhi.ki, gainPhi.kd, gainPhi.fc, _Ts);    

        controlX = _controlX;
        controlY = _controlY;
        controlZ = _controlZ;
        controlPhi = _controlPhi;
    }
    else{
        std::cout << "Invaid AUV control System\n";
        exit(1);
    }
}

void AUV::move(){
    x += vx*Ts;
    y += vy*Ts;
    z += vz*Ts;
    phi += vphi*Ts;
    // std::cout << "move works\n";
}

void AUV::accelerate(){
    vx += fx*Ts;
    vy += fy*Ts;
    vz += fz*Ts;
    vphi += fphi*Ts;
    // std::cout << "accelerate works\n";

}

void AUV::controlResponse(){
        // std::cout << "controlRes works\n";
    fx = controlX->update(refx, x);
    fy = controlY->update(refy, y);
    fz = controlZ->update(refz, z);
    fphi = controlPhi->update(refphi, phi);
}