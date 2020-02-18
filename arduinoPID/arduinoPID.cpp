#include <TimerFive.h>

#define EC_CHA 3  //모터B 채널 A 핀
#define EC_CHB 49  //모터B 채널 B 핀
#define MC_DIR1 42 //모터B 방향1 핀
#define MC_DIR2 43 //모터B 방향2 핀
#define MC_PWM 9  //PWM

#define EB_CHA 19 //모터B 채널 A 핀
#define EB_CHB 38 //모터B 채널 B 핀
#define MB_DIR1 36 //모터B 방향1 핀
#define MB_DIR2 37 //모터B 방향2 핀
#define MB_PWM 8  //PWM

#define EA_CHA 18  //모터B 채널 A 핀
#define EA_CHB 31  //모터B 채널 B 핀
#define MA_DIR1 35 //모터B 방향1 핀
#define MA_DIR2 34 //모터B 방향2 핀
#define MA_PWM 12  //PWM

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

    void Encoder();

public:
    MOTR();
    MOTR(PinNum _nChA, PinNum _nChB, PinNum _nDIR1, PinNum _nDIR2, PinNum _nPWM);
    ~MOTR();

    void setup(PinNum nPin, PinNum nCh, PinNum nDIR1, PinNum nDIR2, PinNum nPWM);
    void setPID(double P, double I, double D);
    void revolving(bool direction, int spd); 

    int getSpd();
};

int encoder_Test1 = 0;
int encoder_Test2 = 0;

void setup() {
    Timer5.initialize(100000); // 타이머 주기 설정
    Serial.begin(9600);

    MOTR A(EA_CHA, EA_CHB, MA_DIR1, MA_DIR2, MA_PWM);
    MOTR B(EB_CHA, EB_CHB, MB_DIR1, MB_DIR2, MB_PWM);
    MOTR C(EC_CHA, EC_CHB, MC_DIR1, MC_DIR2, MC_PWM);

    A.setPID(0.5, 0.1, 3.5);
    B.setPID(0.43, 0.15, 3.5);
    C.setPID(1, 0.5, 0.6);
}

void loop() {
    A.revolving(FWARD, 100);
    B.revolving(BACKWARO, 100);

    A.revolving(FWARD, 0);
    B.revolving(FWARD, 0);
}

MOTR::MOTR(){
    err = 0;
    err_pre = 0;
    I_sum = 0;
}

MOTR::MOTR(PinNum _nChA, PinNum _nChB, PinNum _nDIR1, PinNum _nDIR2, PinNum _nPWM){
    err = 0;
    err_pre = 0
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

    attachInterrupt(digitalPinToInterrupt(pChA), Encoder, CHANGE);
    Timer5.attachInterrupt(getSpd);
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

    encoder_Test2++;
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
    analogWrite(nPWM, speed);
}