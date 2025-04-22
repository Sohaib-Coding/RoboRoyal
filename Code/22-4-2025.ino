#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// التعريفات
int enA = 2;
int in1 = 3;
int in2 = 4;
int in3 = 5;
int in4 = 6;
int enB = 7;

//---Distance_GY-53---
const int dis = 12; 

// حساسات IR (3 حساسات فقط: اليسار، الوسط، اليمين)
int Rs2 = A5;
int Rs = A6;
int Ls = A8;
int Ls2 = A9;
int Ms = A7;

int right2 , right, mid , left , left2 ;
int lm = 255;  // سرعة 100%
int rm = 255;  // سرعة 100%
unsigned long startTime;
bool highSpeed = true;

String lastState = "";

void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  Serial.println(F("لم يتم العثور على الشاشة OLED"));
  while (true); // توقف البرنامج إذا الشاشة غير موجودة
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(45,30);
  display.println("");
  display.display();
  delay(1);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(Rs2, INPUT);
  pinMode(Rs, INPUT);
  pinMode(Ls, INPUT);
  pinMode(Ms, INPUT);
  pinMode(Ls2, INPUT);
  pinMode(dis, INPUT);

  // بدء المؤقت
  startTime = millis();
}


void loop() {

  displayState("Ju'aifas");

  delay(500);
  str();
  displayState("what next ??");
  
  exit (loop);
 }

void str() {

  while (true) {

    right2 = digitalRead(Rs2);
    right = digitalRead(Rs);
    left = digitalRead(Ls);
    left2 = digitalRead(Ls2);
    mid = digitalRead(Ms);
    unsigned long duration = pulseIn(dis, HIGH); // Read pulse duration 
    float distance_mm = duration; // Every microsecond = 1 mm
    int dis_cm = (distance_mm / 100.0) ;

    // الحركة بناءً على الحساسات
    if (right == 1 && left == 1) {
      displayState("FORWARD");
      
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, lm);
      analogWrite(enB, rm);
    } else if (right == 1 && left == 0) {
      displayState("RIGHT");

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enA, 75);
      analogWrite(enB, 75);
    } else if (right == 0 && left == 1) {
      displayState("LEFT");

      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, 75);
      analogWrite(enB, 75);
    }

    // إذا كانت جميع الحساسات 0، توقف
    if ( (right == 0 && left == 0 && mid == 0) ||  (right2 == 0 && right == 0) || (left2 == 0 && left == 0) || (dis_cm < 10) ){
      displayState("str ENDED");

      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(enA, 0);
      analogWrite(enB, 0);
      return;

    }

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

    //  if (dis_cm <= 10){
      //    digitalWrite(in1, LOW);
        //  digitalWrite(in2, LOW);
          //digitalWrite(in3, LOW);
         // digitalWrite(in4, LOW);
          //analogWrite(enA, 0);
          //analogWrite(enB, 0);
          //exit(str);
          //return;
  //} 
}
  

void displayState(String state) {
  if (state != lastState) {
    display.clearDisplay();
    display.setCursor(45, 30);
    display.println(state);
    display.display();
    lastState = state;
  }
}
