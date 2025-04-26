#include <Wire.h>
#include <SoftwareWire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RR_6050.h>

// SCL == A5 && SDL == A4
SoftwareWire myWire(10, 11);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RR_6050 mpu;

const int targetYaw = 90;  // الزاوية المستهدفة

//---H-Bridge---
int enA = 2;
int in1 = 3;
int in2 = 4;
int in3 = 5;
int in4 = 6;
int enB = 7;

//---IR---
// Black = 0 | White = 1 
int Rs = A7; // RIGHT
int Ls = A8; // LEFT
int ls2 = A9; 

//---PID---
float Kp = 32.0;  
float Ki = 0.0;
float Kd = 95.5;

float error = 1 ;
float previousError = 0;
float integral = 0;

//---Distance---
const int dis = 12; 

int black_counter = 0; // hussain runnnnn


int baseSpeed = 150;
int maxSpeed = 235;



//functions
void gold();
void deg180();
void deg90();
void displaystate(String state);
void stop();
void PID_N();
void PID_start();

void setup() {
  Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED IS NOT CONNECTED"));
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(17, 20);
  display.println("Ju'aifas");
  display.display();
  delay(2000);
  display.setTextSize(1);
 
  mpu.begin();
    
  Serial.println("Calibrating gyro... Please keep the sensor still.");
  mpu.calibrate(); // معايرة الجيروسكوب
  Serial.println("Calibration complete!");
 
  mpu.setPrintInterval(10); 

  //---H_Bridge---
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(Rs, INPUT);
  pinMode(Ls, INPUT);
  //---check IR sensors---
  display.clearDisplay();
  display.setCursor(10, 20);
  display.print("Right: ");
  display.print(digitalRead(Rs));
  display.setCursor(10, 35);
  display.print("Left: ");
  display.print(digitalRead(Ls));
  display.display();
}

void loop() {  
  
 
  PID_start();
  displaystate("end 1st PID");
  delay(500);
  

  gold(); 
  delay(1000);
  //put the bluetooth void to pickup the medal
  
  Gyro_180();
  displaystate("180 turned");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  delay(750); 
  stop();

  delay(200);
  PID_N();
  delay(100);

  Gyro90();
  displaystate("Gyro_end");
  delay(500);
  
  PID_N();

  delay(500);
  Gyro_180();
  exit(loop);

}


void PID_start() {
  while (true) {
       
    unsigned long duration = pulseIn(dis, HIGH); // Read pulse duration 
    float distance_mm = duration; // Every microsecond = 1 mm
    int dis_cm = (distance_mm / 100.0) ;


    int right = digitalRead(Rs); 
    int left = digitalRead(Ls);

    //stop 
   if (dis_cm <= 15 )   {
      stop();
      break; // Getout of the loop 
    }

    error = right - left;

    // PID
    integral += error;
    float derivative = error - previousError;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);

    //Start_Moving
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, leftSpeed);
    analogWrite(enB, rightSpeed);

    displaystate("Err: " + String(error));
    delay(5);
  }
}

void PID_N(){
  while (true) {

    int right = digitalRead(Rs); 
    int left = digitalRead(Ls);
    int left2 = digitalRead(ls2);
    //stop 
   if ((right == 1 && left == 1) or (left2 == 1))   {
      stop();
      return; // Getout of the loop 
    }

    error = right - left;

    // PID
    integral += error;
    float derivative = error - previousError;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);

    //Start_Moving
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, leftSpeed);
    analogWrite(enB, rightSpeed);

    displaystate("Err: " + String(error));
    delay(0);
  }
}


void stop() {
  //Simple reverse motion to stop
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 200);
  analogWrite(enB, 200);

  delay(100);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}


void gold(){
  unsigned long duration = pulseIn(dis, HIGH); // Read pulse duration 
  float distance_mm = duration; // Every microsecond = 1 mm
  int dis_cm = (distance_mm / 100.0) ;

  display.setCursor(0, 0);
  display.println("distance: ");
  display.print(dis_cm);
  display.println(" cm");
  display.display();

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 75);
  analogWrite(enB, 75);
  displaystate("back");
  delay(750);

  stop();
  displaystate("stopped");
  while (true) {
    if (dis_cm > 30) {
      PID_N();
  } else {
    if (dis_cm < 30) 
    stop();
    delay(100);
    return;
  }
  return;
}
return;
}



void deg180(){

  while (digitalRead(Rs) == 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, 100); 
    analogWrite(enB, 105);
  }
  delay(200);
  stop180();
  
  return;

  while (true); 
}


void Gyro_180(){
  while (true){
  displaystate("Gyro_start");
  mpu.update();   //update sensor reads

  float currentYaw = mpu.getYaw();  
  displaystate(String(currentYaw));
  Serial.println(" ");
  Serial.print("Yaw:");
  Serial.print(currentYaw);
  Serial.print("°");
  
  if (currentYaw >= 185 && currentYaw <= 270){
    displaystate("Target angle reached!");
    stop180(); //stop  
    break;
  }
  else {
    // 
    if (currentYaw < targetYaw) {
      // if angle > 90 
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, 75); 
      analogWrite(enB, 75 );
      delay(100);
      Serial.println("Rotating clockwise...");
    }

  }
  delay(0);  // تأخير بسيط لتجنب التحديث السريع جداً
}
  return;

}
void stop180(){

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 200);
  analogWrite(enB, 200);

  delay(100);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
void deg90f(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  delay(750); 
  stop();

  while (digitalRead(Rs) == 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, 100); 
    analogWrite(enB, 105);
  }
  delay(200);
  
  stop180();
  PID_N();
  delay(500);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  delay(750); 
  stop();
  delay(500);
  deg180();
  return;
  while (true); 
}




void Gyro90(){
  while (true){
  displaystate("Gyro_start");
  Serial.println("Gyro_start");

  mpu.update();   //update sensor reads
  
  float currentYaw = mpu.getYaw();  
  displaystate(String(currentYaw));
  Serial.println(" ");
  Serial.print("Yaw:");
  Serial.print(currentYaw);
  Serial.print("°");
  
  if (abs(currentYaw >= 270) && (currentYaw <= 340)  ) {
    displaystate("Target angle reached!");
    stop180(); //stop  
    Serial.println("111");
    return;
  }
  else {
    // 
    if (currentYaw < 270) {
      // if angle > 90 
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, 75); 
      analogWrite(enB, 75);
      delay(100);
      Serial.println("Rotating clockwise...");
    }

  }
  delay(0);  // تأخير بسيط لتجنب التحديث السريع جداً
}
  return;
}



void FTR () {
  
  unsigned long duration = pulseIn(dis, HIGH); // Read pulse duration 
  float distance_mm = duration; // Every microsecond = 1 mm
  int dis_cm = (distance_mm / 100.0) ;

  display.setCursor(0, 0);
  display.println("distance: ");
  display.print(dis_cm);
  display.println(" cm");

  display.display();
}



void displaystate(String state) {
  
  display.clearDisplay();
  display.setCursor(20, 32);
  display.println(state);
  display.display();

}
