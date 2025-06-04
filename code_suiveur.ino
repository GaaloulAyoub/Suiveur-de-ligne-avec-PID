#include <QTRSensors.h>



#include <QTRSensors.h>
// Déclaration des pins   vous pouvez utiliser ces pin si vous allez travailler avec 8 capteurs27,26,25,33,32,15,2,4

const int SensorRight =  23;
const int SensorLeft  =  5;
const int R_Motor_1 = 19 ; 
const int L_Motor_1 = 22 ; 
const int RM=14;
const int LM=12;
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2
unsigned int sensorValues[NUM_SENSORS];

QTRSensorsAnalog qtr((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
// Definir les vitesses 
int MIN_SPEED = 0;
int MAX_SPEED = 250;
int REF_SPEED = 70;

// Definir les constantes de correction PID
double kp = 0.3; //Propportionnel
double Ki = 0;   //Integrateur
double Kd = 0; //Dérivateur 

// Definir les variables de correction
int p=0,der=0,lastError=0,I;

// Definir les variables de l'algorithme 
int R,L,a,b,c,d,e,f,g,h; // pour la lecture des capteurs
int wakt,pos,po,virage=0; // position
int chouka=0,depart=0,charge=0;

void setup() {
Serial.begin(9600);
analogReadResolution(10);

//déclaration des Input/Output  
pinMode(SensorRight,INPUT);
pinMode(SensorLeft,INPUT);

pinMode(R_Motor_1,OUTPUT);

pinMode(L_Motor_1,OUTPUT);

pinMode(LM,OUTPUT);
pinMode(RM,OUTPUT);
int i;
for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
{
  qtr.calibrate();    
  analogWrite(R_Motor_1,255);// reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
}
analogWrite(R_Motor_1,0);
// Initialisation de la carte de puissance
digitalWrite(R_Motor_1,LOW);

digitalWrite(L_Motor_1,LOW);

digitalWrite(RM,HIGH);
digitalWrite(LM,HIGH);
delay(3000);

}





void PID_control() 
{
  int position = qtr.readLine(sensorValues);
  int error = 3500 - position; // erreur= consigne - valeur instantané  (3500 si 8RC)
 
   p = error;
  // I = error + I;
   der = error - lastError;
   lastError = error; 

  int motorSpeedChange = p*kp + der*Kd ;//+ I*Ki ; 
  
  analogWrite(R_Motor_1, constrain(REF_SPEED + motorSpeedChange, MIN_SPEED, MAX_SPEED));
  analogWrite(L_Motor_1, constrain(REF_SPEED - motorSpeedChange, MIN_SPEED, MAX_SPEED));
}





void loop() {
start();
wakt=millis();
R=digitalRead(SensorRight); // Capteur Droite(Right)
L=digitalRead(SensorLeft);  // Capteur Gauche(Left )
Serial.print(R);
Serial.print(L);
int i=0;
virage=L*1+R*10;  // presenter l'état de deux capteur en un seul variable afin d'optimiser et de simplifier  
if (virage==00){
  PID_control();
}
else if ((virage==11)){
  analogWrite(L_Motor_1,MAX_SPEED);analogWrite(R_Motor_1, MAX_SPEED);delay(400);
}
else if ((virage==1) ){
  
  analogWrite(L_Motor_1,MIN_SPEED );analogWrite(R_Motor_1, MAX_SPEED);delay(500);// dour aal isar
}
else if ((virage==10)){
  
  analogWrite(L_Motor_1,MAX_SPEED );analogWrite(R_Motor_1, MIN_SPEED);delay(800);// dour aal imin
}
}




void start(){
analogWrite(R_Motor_1,255);
analogWrite(L_Motor_1,255);
delay(400);
analogWrite(R_Motor_1,0);
analogWrite(L_Motor_1,255);
delay(300);
analogWrite(R_Motor_1,255);
analogWrite(L_Motor_1,255);
delay(100);
analogWrite(R_Motor_1,255);
analogWrite(L_Motor_1,0);
delay(700);

}
