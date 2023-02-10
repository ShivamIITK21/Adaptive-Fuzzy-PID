#include "fuzzyPID.hpp"
using namespace fl;

void FuzzyPID::setMembershipFuncsInp(InputVariable * var, double start, double end){
    // var->addTerm(new Triangle("NL", start, start,))
    scalar p1 = start;
    scalar p2 = (end-start)/6 + start;
    scalar p3 = (end-start)/6 + p2;
    scalar p4 = (end-start)/6 + p3;
    scalar p5 = (end-start)/6 + p4;
    scalar p6 = (end-start)/6 + p5;
    scalar p7 = end;

    var->addTerm(new Triangle("NL", p1, p1, p2));
    var->addTerm(new Triangle("NM", p1, p2, p3));
    var->addTerm(new Triangle("NS", p2, p3, p4));
    var->addTerm(new Triangle("Z", p3, p4, p5));
    var->addTerm(new Triangle("PS", p4, p5, p6));
    var->addTerm(new Triangle("PM", p5, p6, p7));
    var->addTerm(new Triangle("PL", p6, p7, p7));
}

void FuzzyPID::setMembershipFuncsOut(OutputVariable * var, double start, double end){
    scalar p1 = start;
    scalar p2 = (end-start)/6 + start;
    scalar p3 = (end-start)/6 + p2;
    scalar p4 = (end-start)/6 + p3;
    scalar p5 = (end-start)/6 + p4;
    scalar p6 = (end-start)/6 + p5;
    scalar p7 = end;

    scalar width = (end-start)/10;
    scalar slope = width*10;

    var->addTerm(new Triangle("NL", p1, width, slope));
    var->addTerm(new Triangle("NM", p2, width, slope));
    var->addTerm(new Triangle("NS", p3, width, slope));
    var->addTerm(new Triangle("Z", p4, width, slope));
    var->addTerm(new Triangle("PS", p5, width, slope));
    var->addTerm(new Triangle("PM", p6, width, slope));
    var->addTerm(new Triangle("PL", p7, width, slope));
}

FuzzyPID::FuzzyPID(std::string param, double _kp, double _ki, double _kd, double _Ts, gainRange _ranges){
    kp = _kp;
    ki = _ki;
    kd = _kd;
    Ts = _Ts;
    ranges = _ranges;
    integral = 0;
    derivative = 0;
    old_ef = 0;

    
    engine = new Engine;
    engine->setName("Gain Controller");
    engine->setDescription("Controls kp, ki, kd");

    e = new InputVariable;
    e->setName("e");
    e->setDescription("");
    e->setEnabled(true);
    e->setRange(ranges.dele.first, ranges.dele.second);
    e->setLockValueInRange(true);
    setMembershipFuncsInp(e, ranges.dele.first, ranges.dele.second);
    engine->addInputVariable(e);


    edot = new InputVariable;
    edot->setName("edot");
    edot->setDescription("");
    edot->setEnabled(true);
    edot->setRange(ranges.deledot.first, ranges.deledot.second);
    edot->setLockValueInRange(true);
    setMembershipFuncsInp(edot, ranges.deledot.first, ranges.deledot.second);
    engine->addInputVariable(edot);
    
    dkp = new OutputVariable;
    dkp->setName("dkp");
    dkp->setDescription("");
    dkp->setEnabled(true);
    dkp->setRange(ranges.delkp.first, ranges.delkp.second);
    dkp->setLockValueInRange(true);
    dkp->setAggregation(new Maximum);
    dkp->setDefuzzifier(new Centroid(100));
    dkp->setDefaultValue(fl::nan);
    dkp->setLockPreviousValue(false);
    setMembershipFuncsOut(dkp, ranges.delkp.first, ranges.delkp.second);
    engine->addOutputVariable(dkp);


    dki = new OutputVariable;
    dki->setName("dki");
    dki->setDescription("");
    dki->setEnabled(true);
    dki->setRange(ranges.delki.first, ranges.delki.second);
    dki->setLockValueInRange(true);
    dki->setAggregation(new Maximum);
    dki->setDefuzzifier(new Centroid(100));
    dki->setDefaultValue(fl::nan);
    dki->setLockPreviousValue(false);
    setMembershipFuncsOut(dki, ranges.delki.first, ranges.delki.second);
    engine->addOutputVariable(dki);

    dkd = new OutputVariable;
    dkd->setName("dkd");
    dkd->setDescription("");
    dkd->setEnabled(true);
    dkd->setRange(ranges.delkd.first, ranges.delkd.second);
    dkd->setLockValueInRange(true);
    dkd->setAggregation(new Maximum);
    dkd->setDefuzzifier(new Centroid(100));
    dkd->setDefaultValue(fl::nan);
    dkd->setLockPreviousValue(false);
    setMembershipFuncsOut(dkd, ranges.delkd.first, ranges.delkd.second);
    engine->addOutputVariable(dkd);

    RuleBlock * kpRules = new RuleBlock;
    kpRules->setName("kpRules");
    kpRules->setDescription("");
    kpRules->setEnabled(true);
    kpRules->setConjunction(fl::null);
    kpRules->setDisjunction(fl::null);
    kpRules->setImplication(new AlgebraicProduct);
    kpRules->setActivation(new General);
    kpRules->addRule(Rule::parse("if e is NL and edot is NL then dkp is PL", engine));
    kpRules->addRule(Rule::parse("if e is NL and edot is NM then dkp is PL", engine));
    kpRules->addRule(Rule::parse("if e is NL and edot is NS then dkp is PM", engine)),
    kpRules->addRule(Rule::parse("if e is NL and edot is Z then dkp is PM", engine));
    kpRules->addRule(Rule::parse("if e is NL and edot is PS then dkp is PS", engine));
    kpRules->addRule(Rule::parse("if e is NL and edot is PM then dkp is Z", engine));
    kpRules->addRule(Rule::parse("if e is NL and edot is PL then dkp is Z", engine));
    kpRules->addRule(Rule::parse("if obstacle is right then mSteer is left", engine));
    engine->addRuleBlock(kpRules);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:\n" + status, FL_AT);

}

double FuzzyPID::update(double ref, double pos){
    double error = ref - pos;
    double new_derivative = (error - old_ef)/Ts;
    double new_integral = integral + error*Ts;

    double updated = kp*error + ki*integral + kd*new_derivative;

    integral = new_integral;
    derivative = new_derivative;
    old_ef = error;

    e->setValue(error);
    edot->setValue(new_derivative);
    engine->process();

    kp += dkp->getValue();
    ki += dki->getValue();
    kd += dkd->getValue();

    return updated;    
}

