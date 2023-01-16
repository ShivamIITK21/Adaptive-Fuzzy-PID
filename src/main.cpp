#include<iostream>
#include "simplePID.hpp"
#include "controllers.hpp"
#include "auv.hpp"

int main(){
    // double start, finish;
    // std::cout << "Enter Starting Pos:";
    // std::cin >> start;
    // std::cout << '\n';
    // std::cout << "Enter Final Pos:";
    // std::cin >> finish;
    // std::cout << '\n';

    // double err = finish - start;
    // double vel = 0;
    // int i = 1;

    // PID control = PID(0.707, 0.1, 1.68, 0.5, 0.1);
    // Controller * ptr;
    // ptr = &control;

    // while(i < 500){
    //     double del = control.update(finish, start);
    //     vel += del*0.1;
    //     start += vel*0.1;
    //     err = finish - start;
        // std::cout << "At " << i*0.1 << " sec, pos=" << start << " ,PID output=" << del << std::endl;
    //     i++;
    // }

    gains X = {0.707, 0.1, 1.68, 0.5};
    gains Y = {0.70, 0.1, 1.68, 0.5};
    gains Z = {0.707, 0.1, 1.6, 0.5};
    gains Phi = {0.707, 0.1, 1.68, 0.4};


    AUV * pid = new AUV("PID", 0, 0, 0, 0, 0.1, X, Y, Z, Phi, 10, 50, 43, 2.16);
    
    int i = 1;
    while(i < 500){
        pid->controlResponse();
        pid->accelerate();
        pid->move();
        std::cout << pid->x << " " << pid->y << " " << pid->z << " " << pid->phi << std::endl;
        i++;
    }
}