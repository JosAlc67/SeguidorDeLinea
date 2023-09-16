/*  los pines a usar son

 Pulsador -- Arduino D12

 Motor A atras --    Arduino D10
 Motor A adelante -- Arduino D5
 Motor B adelante -- Arduino D6
 Motor B atras --    Arduino D9
 
 Sensor derecho     Arduino D2
 Sensor Izquierdo   Arduino D4

 */
byte i;                  // variable global 
void setup() {                
  // Declaracion de pines de entrada y salida 

  pinMode(12, INPUT);     // Pulsador entrada
  pinMode(2, INPUT);      // Sensor Derecho
  pinMode(4, INPUT);      // Sensor izquierdo
  pinMode(10, OUTPUT);     // Motor A Atras
  pinMode(5, OUTPUT);     // Motor A adelante
  pinMode(6, OUTPUT);     // Motor B adelante
  pinMode(9, OUTPUT);     // Motor B Atras
  digitalWrite(12, HIGH);   // Pullup para pulsador

}

void loop() {

inicio:        // etiqueta de inicio del programa por uso de saltos 

analogWrite(5, 0);                                                // Velocidad 0
analogWrite(6, 0);                                                // Velocidad 0
analogWrite(10, 0);                                                // Velocidad 0
analogWrite(9, 0);                                                // Velocidad 0

while (digitalRead(12)==1);                                 // Pulsador para inicio de ejecucion, el programa solo iniciara cuando se pulse
 while (digitalRead(12)==0);                               // una vez sea pulsado se espera a que se deje de presionar para iniciar 
  delay(100);                                                  // retardo de 100ms se evitan rebotes

for(;;)
  {
  if (digitalRead(4) && digitalRead(2))                // Lectura de las señales digitales y ejecucion cuando los dos sensores estan en blanco
  {  analogWrite(5, 80);                                                // Movimiento adelante 
     analogWrite(6, 0);
     analogWrite(10, 0);                                                
     analogWrite(9, 80);
  }
  else if (!digitalRead(2) && digitalRead(4))            // Lectura de las señales digitales y ejecucion cuando el sensor izquierdo entra a la linea negra
  {  analogWrite(5, 23);                                                
     analogWrite(6, 23);
     analogWrite(10, 0);                                                 
     analogWrite(9, 0);
  }                                              
  else if (digitalRead(2) && !digitalRead(4))           // Lectura de las señales digitales y ejecucion cuando el sensor derecho entra a la linea negra
  {  analogWrite(5, 0);                                                
     analogWrite(6, 0);
     analogWrite(10, 23);                                                 
     analogWrite(9, 23);
  }  
  else                                                         // Lectura de las señales digitales y ejecucion cuando los dos sensores estan en linea negra
  {  analogWrite(5, 0);                                                 
     analogWrite(6, 0);
     analogWrite(10, 0);                                                 
     analogWrite(9, 0);
  }                                                                                                      
  if (digitalRead(12)==0)                                      // Lectura del pulsador de arranque y parada
  {  while (digitalRead(12)==0);                               // Si fue pulsado espera a que se deje de precionar y para 
      delay(100);                                              // retardo de 100ms se evitan rebotes
    goto inicio;}                                              // Vuelve a inicio a esperar pulso para arranque
  }

}
