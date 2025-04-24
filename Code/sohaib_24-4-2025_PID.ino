#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// المحركات
int enA = 2;
int in1 = 3;
int in2 = 4;
int in3 = 5;
int in4 = 6;
int enB = 7;

// الحساسات
int Rs = A7; // يمين
int Ls = A8; // يسار

// PID
float Kp = 32.0;  // حساسية التعديل
float Ki = 0.0;
float Kd = 95.5;

const int dis = 12; 

float error = 1 ;
float previousError = 0;
float integral = 0;

int baseSpeed = 180;
int maxSpeed = 255;

void setup() {
  Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("لم يتم العثور على الشاشة OLED"));
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);
  display.println("");
  display.display();
  delay(1000);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(Rs, INPUT);
  pinMode(Ls, INPUT);
}

void loop() {
  
  displayState("Ju'aifas");
  delay(1000);
  
  PID();
  delay(2000);
  displayState("end");
  PID();
  delay(1000);
  exit(loop);
}

void line1() {
  while (true){
    if (digitalRead(Ls) == 0 || digitalRead(Rs) == 1){
      displayState("PID Start");
      delay(1000);
      PID();
      
    }else {
      if (Ls == 0 && Rs == 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        analogWrite(enA, 0 );
        analogWrite(enB, 0 );   
        displayState("PID Ends");
        return;
      }
    

  }
}
}

void PID() {
  while (true) {
       
    unsigned long duration = pulseIn(dis, HIGH); // Read pulse duration 
    float distance_mm = duration; // Every microsecond = 1 mm
    int dis_cm = (distance_mm / 100.0) ;

    display.setCursor(0, 0);
    display.println("distance: ");
    display.println(dis_cm );
    display.println(" cm ");
    display.display();


    int right = digitalRead(Rs); // 1: أبيض | 0: أسود
    int left = digitalRead(Ls);

    // التوقف عند التقاطع (كلا الحساسين على الخط)
    if (right == 1 && left == 1 || dis_cm <= 20) {
      // توقف المحركات
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(enA, 0);
      analogWrite(enB, 0);
      displayState("Stop: Cross Detected");
      break; // خروج من PID والعودة إلى line1()
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

    // حركة للأمام
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, leftSpeed);
    analogWrite(enB, rightSpeed);

    displayState("Err: " + String(error));
    delay(20);
  }
}

  
void FTR () {
  
  unsigned long duration = pulseIn(dis, HIGH); // Read pulse duration 
  float distance_mm = duration; // Every microsecond = 1 mm
  int dis_cm = (distance_mm / 100.0) ;

  display.setCursor(0, 0);
  display.println("distance: ");
  display.println(dis_cm );
  display.println(" cm ");
  display.display();
}
  void displayState(String state) {
    display.clearDisplay();
    display.setCursor(10, 30);
    display.println(state);
    display.display();
}
