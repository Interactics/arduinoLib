#include "pidcntl.h"
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

void setup() {
    Serial.begin(9600);

    MOTR A(EA_CHA, EA_CHB, MA_DIR1, MA_DIR2, MA_PWM);
    MOTR B(EB_CHA, EB_CHB, MB_DIR1, MB_DIR2, MB_PWM);
    MOTR C(EC_CHA, EC_CHB, MC_DIR1, MC_DIR2, MC_PWM);

    A.setPID(0.5, 0.1, 3.5);
    B.setPID(0.43, 0.15, 3.5);
    C.setPID(1, 0.5, 0.6);

    attachInterrupt(digitalPinToInterrupt(A.pChA), A.Encoder, A.CHANGE);
    attachInterrupt(digitalPinToInterrupt(B.pChA), B.Encoder, B.CHANGE);
    attachInterrupt(digitalPinToInterrupt(C.pChA), C.Encoder, C.CHANGE);

    Timer5.attachInterrupt(getSpeed);
    Timer5.initialize(100000); // 타이머 주기 설정

}

void loop() {
    A.revolving(FWARD, 100);
    B.revolving(BACKWARO, 100);
    delay(1000);
    A.revolving(FWARD, 0);
    B.revolving(FWARD, 0);
    delay(1000);
}

void getSpeed(){
  A.getSpd();
  B.getSpd();
  C.getSpd();
}