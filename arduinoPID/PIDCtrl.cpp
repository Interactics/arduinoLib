#include "pidcntl.h"
#include <Arduino.h>

MOTR::MOTR(){
    err = 0;
    err_pre = 0;
    I_sum = 0;
}

MOTR::MOTR(PinNum _nChA, PinNum _nChB, PinNum _nDIR1, PinNum _nDIR2, PinNum _nPWM){
    err = 0;
    err_pre = 0;
    I_sum = 0;
    
    setup(_nChA, _nChB, _nDIR1, _nDIR2, _nPWM);
}

MOTR::~MOTR(){}

//Pin Number Setting
void MOTR::setup(PinNum _nChA, PinNum _nChB, PinNum _nDIR1, PinNum _nDIR2, PinNum _nPWM){
    pChA = _nChA;
    pChB = _nChB;
    pDIR1 = _nDIR1;
    pDIR2 = _nDIR2;
    pPWM = _nPWM;

    pinMode(pChA,INPUT);
    pinMode(pChB,INPUT);
    pinMode(pDIR1,OUTPUT);
    pinMode(pDIR2,OUTPUT);
    pinMode(pPWM,OUTPUT);

    //해결해야할 부분.
    // 따로 빼내자.
}

//Setting PID value
void MOTR::setPID(const double P, const double I, const double D){
    P_gain = P;
    I_gain = I;
    D_gain = D;
}

void MOTR::move(double targetSpd){
    target = targetSpd;
}

void MOTR::Encoder(){
    if(digitalRead(pChA) == digitalRead(pChB))
        encoder++;
    else
        encoder--;
}

void MOTR::GetSpd(){
    speed = encoder * Wheel_D * pi / 663 / 10;
    encoder = 0;
    pwm = PID(abs(target), abs(speed));
}

int MOTR::PID(const double target, const double now){
    double P, D;
    
    err = 150 * (target - now);
    P = err * P_gain;
    D = (err - err_pre) * 0.1 * D_gain;
    
    if (I_sum <= (target * 130 + 40))
        I_sum += (err_pre + err) /2 * 0.1 * I_gain;
    if (I_sum >= (target * 130 + 40))
        I_sum -= abs(err_pre + err)/2*0.1*I_gain;
    err_pre = err;

    if(P + D + I_sum > 255) return 255;
    else return P + D + I_sum;
}

void MOTR::revolving(bool direction, int spd){
    speed = spd;

    digitalWrite(pDIR1, direction);
    digitalWrite(pDIR2, !direction);
    analogWrite(pPWM, speed);
}
