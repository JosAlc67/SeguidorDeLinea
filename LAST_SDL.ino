#include <QTRSensors.h>

QTRSensors qtr;
const int btn1 = 12;
const int btn2 = 4;
int cruzero = 50;
int ain1 = 3;
int pwma = 5;
int ain2 = 4;
int pwmb = 6;
int bin2 = 7;
int bin1 = 8;
int STANBAY = 9;

int P = 0;
int I = 0;
int D = 0;
int LAST = 0;
float vel;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
unsigned int position = 0;

void setup() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6,A7}, SensorCount);
    qtr.setEmitterPin(2);

    pinMode(btn1, INPUT);

    pinMode(pwma, OUTPUT);
    pinMode(ain1, OUTPUT);
    pinMode(ain2, OUTPUT);
    pinMode(pwmb, OUTPUT);
    pinMode(bin1, OUTPUT);
    pinMode(bin2, OUTPUT);
    pinMode(STANBAY, OUTPUT);

    pinMode(13, OUTPUT);

    delay(1500);
    digitalWrite(13, HIGH);

    for (uint16_t i = 0; i < 150; i++) {
        qtr.calibrate();
    }

    digitalWrite(13, LOW);

    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);

    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);

    analogWrite(pwma, 0);
    analogWrite(pwmb, 0);
}

void loop() {
    digitalWrite(STANBAY, HIGH);

    uint16_t position = qtr.readLineBlack(sensorValues);

    P = (position - 3500);

    // Ajuste de velocidad en curvas
    adjustSpeedForCurves();

    //if (-500<= P <=500){
      //advance();
    if (P <= -3200) {
       applyBrakes();
      
    } else if (P >= 3200) {
        reverseMotion();
    } else {
        calculatePID();
    }
}
void advance(){
    analogWrite(pwmb, 150);
    analogWrite(pwma, 150);
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
}

void applyBrakes() { //giro a la derecha
    analogWrite(pwmb, 50);//90
    analogWrite(pwma, 75);//130
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
}

void reverseMotion() {  //giro a la izquierda
    analogWrite(pwmb, 75); //130
    analogWrite(pwma, 50); //90
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
}

void calculatePID() {
    D = (P - LAST);
    I = (I + P);

    // Ajuste de las constantes PID
    float Kp = 0.08 ;//0.4; o   0.08
    float Ki = 0.0005; // 0.00045
    float Kd = 0.1; //0.5;

    // Escalado de la contribución de la parte integral
    I *= Ki;

    // Saturación integral anti-windup
    float integralLimit = 100;  // Ajusta según sea necesario
    if (I > integralLimit) I = integralLimit;
    else if (I < -integralLimit) I = -integralLimit;

    // Cálculo final del control PID
    vel = (P * Kp) + (D * Kd) + I;


    // Aplicación del control a los motores
    analogWrite(pwmb, cruzero - vel);
    analogWrite(pwma, cruzero + vel);

    // Configuración de direcciones de los motores
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);

    // Actualización del valor anterior
    LAST = P;
}



void adjustSpeedForCurves() {
    if (abs(P) > 1000) {
        cruzero = 70;//180..120
    } else {
        cruzero = 50;  // 160 Velocidad normal
    }
}
