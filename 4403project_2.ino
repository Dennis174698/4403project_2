#include <Wire.h> //I2C Arduino Library
#define addr 0x1E 
#include <stdlib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include<Servo.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
#define Trig 2 //D2
#define Echo 3 //D3
const int analogInPin = 14;  // sensor input pin 
const int servoOutpin = 9;   // servo output pin 
int sensorValue = 0;        // sensor value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
//int servoBefore = 0;         // value output to the PWM (analog out)
//int servoAfter = 0;
float cm;
float temp;
int servoValue = 0;
int motorOutpin = 7;         // morotr pin
int motorOutpin2 = 6;        
int motorOutpin3 = 5;
int motorOutpin4 = 4;       //
float init_angle = 0;       // initial angle       
float angle = 0 ;          // angle
float turnAngle = 0;    //
boolean flag = false;              //Uturn done or not

Servo myservo;


float compassRead(float heading){
  /* Get a new sensor event */ 
  sensors_event_t   event; 
  mag.getEvent(&event);
  
  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
  heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  return heading;
}

void set_init(){
  init_angle = compassRead(angle);
  myservo.write(90);
  delay(500);
  
}

void Uturn2(){
  flag = false;
  //digitalWrite(motorOutpin,LOW);
  //digitalWrite(motorOutpin2,LOW);
  
  //digitalWrite(motorOutpin,HIGH);
  //digitalWrite(motorOutpin2,LOW);
  //digitalWrite(motorOutpin3,HIGH);
  
  
  myservo.write(120);
  delay(100);
  myservo.write(140);
  delay(100);
  myservo.write(160);
  delay(3600);
  init_angle = init_angle+180;
  flag=true;
}
void distance(){
  digitalWrite(Trig, LOW); 
  delayMicroseconds(2);    
  digitalWrite(Trig,HIGH); 
  delayMicroseconds(10);    
  digitalWrite(Trig, LOW); 
  
  temp = float(pulseIn(Echo, HIGH)); 
  //calculte the time between HIGH to LOW
  
  
  //34000cm / 1000000μs => 34 / 1000
  //
  //distance(cm)  =  (temp * (34 / 1000)) / 2
  // (temp * 17)/ 1000
  
  cm = (temp * 17 )/1000; //transfer to cm
 
  Serial.print("Echo =");
  Serial.print(temp);
  Serial.print(" | | Distance = ");
  Serial.print(cm);
  Serial.println("cm");
  //delay(100);

}

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Serial.println();
  pinMode(Trig,OUTPUT); //D2
  pinMode(Echo,INPUT);  //D3
  pinMode(motorOutpin,OUTPUT);
  pinMode(servoOutpin, OUTPUT);
  pinMode(motorOutpin2,OUTPUT);
  pinMode(motorOutpin3,OUTPUT);
  pinMode(motorOutpin4,OUTPUT);
  myservo.attach(servoOutpin);
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    
  }
  set_init();
}


void loop() {

  angle = compassRead(angle);
  distance();//read distance
  // print the results to the Serial Monitor:
  
  
  Serial.print("Compass Heading: ");
  Serial.println(angle);
  //delay(100);
  
  digitalWrite(motorOutpin,HIGH);
  digitalWrite(motorOutpin2,HIGH);
  digitalWrite(motorOutpin3,LOW);
  digitalWrite(motorOutpin4,LOW);
  delay(50);
  if(cm<45)
  {
    distance();
    while(cm<=45&&flag&&servoValue<=130&&servoValue>=60){
      digitalWrite(motorOutpin,LOW);      //reverse
      digitalWrite(motorOutpin2,LOW);
      digitalWrite(motorOutpin3,HIGH);
      digitalWrite(motorOutpin4,HIGH);
      delay(2000);
      while(1){
      digitalWrite(motorOutpin,LOW);
      digitalWrite(motorOutpin2,LOW);
      digitalWrite(motorOutpin3,LOW);
      digitalWrite(motorOutpin4,LOW);
      
      }
      
    }
    if(!flag){
    Uturn2();
    //digitalWrite(Trig, LOW);
    //delay(200);
    //digitalWrite(Trig,HIGH);
    }      
  }
    
    turnAngle = int(init_angle)-int(angle);
    if(turnAngle>180){
      turnAngle = int(angle)-int(init_angle)+360;
      servoValue = 90+turnAngle;
    }
    else if(turnAngle<=-180){
       turnAngle+=360;
       servoValue = 90-turnAngle;
    }
    else if(turnAngle>-180&&turnAngle<180)
    {
      servoValue = 90-turnAngle;
    }
    Serial.println(init_angle);
    Serial.println(servoValue);
    if(servoValue>=180){
      
      servoValue = 160;
    }
    if(servoValue<=0){
      servoValue = 20;
    }
    myservo.write(servoValue);
    Serial.println(servoValue);
    delay(1);
    
    //myservo.write(180);
    //delay(1000);
  
  


  
}
