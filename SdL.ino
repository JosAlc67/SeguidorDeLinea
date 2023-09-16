#include <OrangutanMotors.h>
#include <QTRSensors.h>

// Definir pines para QTR-8 sensor
#define NUM_SENSORS  8  // número de sensores usados
#define NUM_SAMPLES_PER_SENSOR 5  // tiempo máximo para completar la lectura del sensor
#define EMITTER_PIN   2  // Pin para emisor IR (opcional)

// Inicializar sensor de línea
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
OrangutanMotors motors;

const int maximum = 180;

int VProporcional = 5; //Últimos provados> 8 proporcional, 6 derivativo.

int VDerivativo = 3; // Funcionó bastante mejor con los últimos provados,
//                      pero los mnotores aún van más rápido que el error porcentual.

int velcalibrate = 24;


void setup()
{
  int inPin = 10;
  int val = 0;
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(inPin, INPUT_PULLUP);

  val = digitalRead(inPin);
  while (val == HIGH)
    {
      digitalWrite(9,HIGH);
      digitalWrite(8,HIGH);
      val = digitalRead(inPin);
    };
  if (val == LOW)
    {
      digitalWrite(9,LOW);
      digitalWrite(8,LOW);
    };
  motors.setSpeeds(0,0);

  delay(1500);
  digitalWrite(8, HIGH); 

  
  
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }

  
  digitalWrite(8, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtra.calibratedMinimumOn[i];
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtra.calibratedMaximumOn[i];
  }
  Serial.println();
  Serial.println();
  delay(1000);

  val = digitalRead(inPin);

  while (val == HIGH)
    {
      digitalWrite(8, HIGH);
      delay(200);
      digitalWrite(8, LOW);
      delay(200);
      val = digitalRead(inPin);
      
    };

/*
  digitalWrite(9,HIGH);
  digitalWrite(8,HIGH);*/

 /* for (int counter = 0; counter<21; counter++)
  {
    if (counter<6||counter>=15)
    OrangutanMotors::setSpeeds(-velcalibrate, velcalibrate);
    else
    OrangutanMotors::setSpeeds(velcalibrate,-velcalibrate);
    qtra.calibrate();
    delay(500);
  }*/
/*
  digitalWrite(9,LOW);
  digitalWrite(8,LOW);

  

  delay(200);

  digitalWrite(9,HIGH);
  digitalWrite(8,HIGH);

  delay(200);
  
  digitalWrite(9,LOW);
  digitalWrite(8,LOW);

  delay(200);

  digitalWrite(9,HIGH);
  digitalWrite(8,HIGH);
  delay(200);

  digitalWrite(9,LOW);
  digitalWrite(8,LOW);
  delay(200);
*/
  /*while (val == HIGH)
  {
    digitalWrite(9,HIGH);
    digitalWrite(8,HIGH);
    val = digitalRead(inPin);
  };
  if (val == LOW)
  {
    digitalWrite(9,LOW);
    digitalWrite(8,LOW);
    delay(1000);
  };*/
  
}

unsigned int last_proportional = 0;
long integral = 0;

void loop() {
    unsigned int position = qtra.readLine(map(sensorValues, 0, 1000, 1000, 0));

    int proportional = (int)position - 2500;

    int derivative = proportional - last_proportional;
    integral += proportional;

    int power_difference = proportional/VProporcional + integral*0 + derivative*VDerivativo;

    if(power_difference > maximum)
    power_difference = maximum;

    if(power_difference < -maximum)
    power_difference = -maximum;

    if(power_difference < 0)
      OrangutanMotors::setSpeeds(maximum, maximum + power_difference);
    else
      OrangutanMotors::setSpeeds(maximum - power_difference, maximum);

    delay(1);
}
