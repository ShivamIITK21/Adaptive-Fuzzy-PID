#include "fuzzyPID.hpp"

FuzzyPID::FuzzyPID(std::string param, double _kp, double _ki, double _kd, double _Ts, gainRange _ranges){
    kp = _kp;
    ki = _ki;
    kd = _kd;
    Ts = _Ts;
    ranges = _ranges;
    integral = 0;
    derivative = 0;
    old_ef = 0;

    using namespace fl;
    engine = new Engine;
    engine->setName("Gain Controller");
    engine->setDescription("Controls kp, ki, kd");

    InputVariable * e = new InputVariable;
    e->setName("Error");
    e->setDescription("");
    e->setEnabled(true);
    e->setRange(ranges.dele.first, ranges.dele.second);
    e->setLockValueInRange(false);
    engine->addInputVariable(e);


    InputVariable * e = new InputVariable;
    e->setName("Error Rate");
    e->setDescription("");
    e->setEnabled(true);
    e->setRange(ranges.deledot.first, ranges.deledot.second);
    e->setLockValueInRange(false);


    engine->addInputVariable(e);
    
    OutputVariable * dkp = new OutputVariable;
    dkp->setName("Delta Kp");
    dkp->setDescription("");
    dkp->setEnabled(true);
    dkp->setRange(ranges.delkp.first, ranges.delkp.second);
    dkp->setLockValueInRange(true);
    dkp->setAggregation(new Maximum);
    dkp->setDefuzzifier(new Centroid(100));
    dkp->setDefaultValue(fl::nan);
    dkp->setLockPreviousValue(false);

    engine->addOutputVariable(dkp);


    OutputVariable * dki = new OutputVariable;
    dki->setName("Delta Ki");
    dki->setDescription("");
    dki->setEnabled(true);
    dki->setRange(ranges.delki.first, ranges.delki.second);
    dki->setLockValueInRange(true);
    dki->setAggregation(new Maximum);
    dki->setDefuzzifier(new Centroid(100));
    dki->setDefaultValue(fl::nan);
    dki->setLockPreviousValue(false);

    engine->addOutputVariable(dki);



    OutputVariable * dkd = new OutputVariable;
    dkd->setName("Delta Kp");
    dkd->setDescription("");
    dkd->setEnabled(true);
    dkd->setRange(ranges.delkd.first, ranges.delkd.second);
    dkd->setLockValueInRange(true);
    dkd->setAggregation(new Maximum);
    dkd->setDefuzzifier(new Centroid(100));
    dkd->setDefaultValue(fl::nan);
    dkd->setLockPreviousValue(false);

    engine->addOutputVariable(dkd);


    

    
}

double FuzzyPID::update(double ref, double pos){
    double error = ref - pos;
    double new_derivative = (error - old_ef)/Ts;
    double new_integral = integral + error*Ts;

    double updated = kp*error + ki*integral + kd*new_derivative;

    integral = new_integral;
    derivative = new_derivative;
    old_ef = error;

    return updated;    
}

