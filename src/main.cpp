#include<iostream>
#include "simplePID.hpp"
#include "controllers.hpp"

int main(){
    double start, finish;
    std::cout << "Enter Starting Pos:";
    std::cin >> start;
    std::cout << '\n';
    std::cout << "Enter Final Pos:";
    std::cin >> finish;
    std::cout << '\n';

    double err = finish - start;
    double vel = 0;
    int i = 1;

    PID control = PID(0.707, 0.1, 1.68, 0.5, 0.1);
    Controller * ptr;
    ptr = &control;

    while(i < 500){
        double del = control.update(finish, start);
        vel += del*0.1;
        start += vel*0.1;
        err = finish - start;
        std::cout << "At " << i*0.1 << " sec, pos=" << start << " ,PID output=" << del << std::endl;
        i++;
    }
}