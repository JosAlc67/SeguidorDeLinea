float sensor1=A0;
float sensor2=A1;
float sensor3=A2;
int sensores[3];
int valores[3];
double ponderado=0;
int boton=2;
int led1=3;
int led2=4;
double posicion;
double pos_sensores[3]={2000,1000,0};
//Motor izquierdo
int M1A=9;
int M1B=8;
int pwm1=5;
//Motor derecho
int M2A=7;
int M2B=6;
int pwm2=10;
int valor;
int cont;

int max=170;    //120 200
float KP=0.8; //0.6 0.9 
float KD=0.6;  //0.3 0.6
int error;
int ult_error;
int derivative;

void setup() {
  // put your setup code here, to run once:
  pinMode(sensor1,INPUT);
  pinMode(sensor2,INPUT);
  pinMode(sensor3,INPUT);
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(M1A,OUTPUT);
  pinMode(M2A,OUTPUT);
  pinMode(M1B,OUTPUT);
  pinMode(M2B,OUTPUT);  
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);
  pinMode(boton,INPUT_PULLUP);
  Serial.begin(9600);
  digitalWrite(led1,HIGH);
  digitalWrite(led2,HIGH);
  while (digitalRead(boton));//espera a que presione el pulsador para continuar
  delay(500);

  //digitalWrite(led,LOW);
  //CALIBRAR SENSORES
  for(int i=0;i<200;i++)
  {
    digitalWrite(led1,HIGH);
     digitalWrite(led2,LOW);
    delay(20);
    leer_sensores();
    digitalWrite(led1,LOW);
     digitalWrite(led2,HIGH);
    delay(20);
  }
  Serial.println(ponderado);
  while (digitalRead(boton));//ESPERAR HASTA PULSAR PARA INICIAR 
  digitalWrite(led1,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  double suma1=0;
  double suma2=0;
  digitalWrite(led1,HIGH);
  if(analogRead(sensor1)>ponderado)valores[0]=1;
  else if (analogRead(sensor1)<ponderado)valores[0]=0;
    if(analogRead(sensor2)>ponderado)valores[1]=1;
  else if (analogRead(sensor2)<ponderado)valores[1]=0;
  
  if(analogRead(sensor3)>ponderado)valores[2]=1;
  else if (analogRead(sensor3)<ponderado)valores[2]=0;
  suma2=valores[0]+valores[1]+valores[2];
  
  for(int i=0;i<2;i++)
  {
    suma1=suma1+valores[i]*pos_sensores[i];
  }
  posicion=suma1/suma2;
  Serial.println(posicion);
  error=(int(posicion)-1000);
  derivative=error-ult_error;
  ult_error=error;
  int PD=(error*KP)+(derivative*KD);
  if(PD>max)PD=max;
  else if(PD<(-max))PD=(-max);
  if(PD < 0) velmotor(max,max+PD);//funcion
  else velmotor(max-PD,max);//funcion
}
  //motores velocidad izquierda puente H
void velizq(int value)
{
  if(value>=0)
  {
    digitalWrite(M1A,HIGH);
    digitalWrite(M1B,LOW);

  }
  else
  {
    digitalWrite(M1A,LOW);
    digitalWrite(M1B,HIGH);
    value*=-1;
  }
  if(value<0)value*=-1;
  analogWrite(pwm1,value);
}
//motores velocidad derecha
void velder(int value)
{
  if(value>=0)
  {
    digitalWrite(M2A,HIGH);
    digitalWrite(M2B,LOW);
  }
  else
  {
    digitalWrite(M2A,LOW);
    digitalWrite(M2B,HIGH);
    value*=-1;
  } 
  if(value<0)value*=-1;
  analogWrite(pwm2,value);
  
}
//velocidad motores
void velmotor(int izq, int der)
{
  velizq(izq);
  velder(der);
}

void leer_sensores(){
  int vmax=0;
  int vmin=1024;
  sensores[0]=analogRead(sensor1);         
  if(sensores[0]<vmin){vmin=sensores[0];}                
  else if(sensores[0] > vmax){vmax=sensores[0];}  
  
  sensores[1]=analogRead(sensor2);         
  if(sensores[1]<vmin){vmin=sensores[1];}                
  else if(sensores[1] > vmax){vmax=sensores[1];} 
  
  sensores[2]=analogRead(sensor3);         
  if(sensores[2]<vmin){vmin=sensores[2];}                
  else if(sensores[2] > vmax){vmax=sensores[2];} 
  ponderado=(vmax+vmin)/2;
    
}
