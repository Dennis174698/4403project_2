#include <Wire.h> //I2C Arduino Library
#define addr 0x1E 
#include <stdlib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include<Servo.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

const int analogInPin = 14;  // sensor input pin 
const int servoOutpin = 9;   // servo output pin 
int sensorValue = 0;        // sensor value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
//int servoBefore = 0;         // value output to the PWM (analog out)
//int servoAfter = 0;
int servoValue = 0;
int motorOutpin = 7;         // morotr pin
int motorOutpin2 = 6;        
int motorOutpin3 = 5;
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
void Uturn(){
  digitalWrite(motorOutpin,LOW);
  delay(1000);
  digitalWrite(motorOutpin,HIGH);
  myservo.write(80);
  delay(500);
  myservo.write(120);
  delay(500);
  myservo.write(140);
  delay(500);
  myservo.write(160);
  
  delay(15000);
  init_angle = init_angle+180;
  flag=1;
}
void Uturn2(){
  flag = 0;
  digitalWrite(motorOutpin,LOW);
  digitalWrite(motorOutpin2,LOW);
  
  digitalWrite(motorOutpin,HIGH);
  digitalWrite(motorOutpin2,LOW);
  digitalWrite(motorOutpin3,HIGH);
  myservo.write(80);
  delay(500);
  myservo.write(120);
  delay(500);
  myservo.write(140);
  delay(500);
  myservo.write(160);
  delay(9000);
  init_angle = init_angle+180;
  flag=1;
}

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Serial.println();

  pinMode(motorOutpin,OUTPUT);
  pinMode(servoOutpin, OUTPUT);
  pinMode(motorOutpin2,OUTPUT);
  pinMode(motorOutpin3,OUTPUT);
  myservo.attach(servoOutpin);
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    
  }
  set_init();
}


void loop() {
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = sensorValue/4;
  // change the analog out value:
  angle = compassRead(angle);

  // print the results to the Serial Monitor:
  
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(outputValue);
  
  Serial.print("Compass Heading: ");
  Serial.println(angle);
  delay(100);
  
  digitalWrite(motorOutpin,HIGH);
  digitalWrite(motorOutpin2,HIGH);
  digitalWrite(motorOutpin3,LOW);
  delay(50);
  if(sensorValue<=250&&sensorValue>=150)
  {
    while(flag==1){
      digitalWrite(motorOutpin,LOW);
      digitalWrite(motorOutpin2,LOW);
      digitalWrite(motorOutpin3,LOW);
    }
    Uturn2();   
  }
    
    turnAngle = int(init_angle)-int(angle);
    if(turnAngle>180){
      turnAngle = int(angle)-int(init_angle)+360;
      servoValue = 90-turnAngle;
    }
    if(turnAngle<=-180){
       turnAngle+=360;
       servoValue = 90+turnAngle;
    }
    else{
      servoValue = turnAngle+90;
    }
    Serial.println(init_angle);
    Serial.println(servoValue);
    if(servoValue>=180){
      
      servoValue = 180;
    }
    if(servoValue<=0){
      servoValue = 0;
    }
    myservo.write(servoValue);
    Serial.println(servoValue);
    delay(1);
    
    //myservo.write(180);
    //delay(1000);
  
  


  
}
