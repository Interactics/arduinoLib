#ifndef PIDCNTL_H_
#define PIDCNTL_H_

#define FWARD 0
#define BACKWARO 1

typedef const int PinNum;

const double Wheel_D =13;
const double pi =3.14;

class MOTR{
    int pChA, pChB;
    int pDIR1, pDIR2;
    int pPWM;

    double P_gain;
    double I_gain;
    double D_gain;

    double err_pre;
    double err;
    double I_sum;
    
    volatile int ChA;
    volatile int ChB;
    volatile int pwm;
    volatile int encoder;

    double speed;
    double target;

public:
    MOTR();
    MOTR(PinNum _nChA, PinNum _nChB, PinNum _nDIR1, PinNum _nDIR2, PinNum _nPWM);
    ~MOTR();

    PinNum pin() { return pChA; }

    void setup(PinNum nPin, PinNum nCh, PinNum nDIR1, PinNum nDIR2, PinNum nPWM);
    void setPID(double P, double I, double D);
    void Encoder();
    void revolving(bool direction, int spd); 

    int getSpd();
};
#endif