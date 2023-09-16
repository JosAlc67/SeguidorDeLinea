#include <OrangutanMotors.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

// Definir pines RX y TX para el modulo Bluetooth
//#define BT_RX 10
//#define BT_TX 11

// Inicializar modulo bluetooth
//SoftwareSerial BTSerial(BT_RX, BT_TX);

// Inicializar motores
OrangutanMotors motors;

// Definir pines para QTR-8 sensor
#define NUM_SENSORS  8  // número de sensores usados
#define TIMEOUT       2500  // tiempo máximo para completar la lectura del sensor
#define EMITTER_PIN   2  // Pin para emisor IR (opcional)

// Inicializar sensor de línea
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  delay(500);
/*  BTSerial.begin(9600);  // Iniciar Bluetooth
*/}

void loop()
{
/*  char c;
  if (BTSerial.available())
    c = BTSerial.read();  // Leer dato de Bluetooth

  if (c == '1') {  // Si recibe '1', empezar a seguir la línea
*/    int position = qtra.readLine(sensorValues);  // Leer la posición de la línea

    int error = position - 1000;  // Calcular el error

    int motorSpeed = 100 + error / 20;  // Calcular la velocidad del motor
    int motorSpeedR = motorSpeed - error;  // Velocidad para el motor derecho
    int motorSpeedL = motorSpeed + error;  // Velocidad para el motor izquierdo

    // Controlar los motores
    motors.setSpeeds(motorSpeedL, motorSpeedR);
//  }
//  else if (c == '0') {  // Si recibe '0', parar los motores
//    motors.setSpeeds(0, 0);
//printf(Integer.toString(sensorValues));
printf(position);
}
//}
