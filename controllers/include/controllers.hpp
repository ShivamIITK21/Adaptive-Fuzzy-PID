#ifndef CONTROLLER_H
#define CONTROLLER_H


class Controller{
    public:
        virtual double update(double ref, double pos) = 0;
};

#endif