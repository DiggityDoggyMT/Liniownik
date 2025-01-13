
#include <QTRSensors.h>




// definiowanie pinow
#define MOTOR_A_IN1 A1  // Lewy silnik 
#define MOTOR_A_IN2 A2  // Lewy silnik 
#define MOTOR_B_IN1 A3  // Prawy silnik 
#define MOTOR_B_IN2 A4  // Prawy silnik 
#define BUTTON_PIN  A5 // PRZYCISK
#define TIME_LIMIT 10000  
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
bool buttonState = digitalRead(BUTTON_PIN); // Odczyt stanu przycisku
int clickCount = 0;           // Licznik kliknięć
unsigned long startTime = 0;  // Czas startu odliczania
bool lastButtonState = HIGH;  // Poprzedni stan przycisku




 void setup()
{
 Serial.begin(9600);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 


  
  
// configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){6, 7, 8, 9, 10, 11, 12, 13}, 8);
  qtr.setEmitterPin(2);

 
   int i;
  for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
  }


// ZATRZYMANIE ROBOTA
void stopMotors() {

  analogWrite(MOTOR_A_IN1, 0);
  analogWrite(MOTOR_A_IN2, 0);
  analogWrite(MOTOR_B_IN1, 0);
  analogWrite(MOTOR_B_IN2, 0);
}

// LEWY SILNIK
void leftMotor(int speed, bool forward) {
  if (forward) {
    analogWrite(2, 0); //Silnik A - obroty w lewo
    analogWrite(3, speed); 
  } else {
    analogWrite(2, speed); //Silnik A - obroty w prawo
    analogWrite(3, 0); 
  }
}

// PRAWY SILNIK
void rightMotor(int speed, bool forward) {
  if (forward) {
    analogWrite(4, 0); //Silnik B - obroty w lewo
    analogWrite(5, speed); 
  } else {
    analogWrite(4, speed); //Silnik B - obroty w prawo
    analogWrite(5, 0); 
  }
}

//TRYB LINE FOLLOWER
void lineFollower()
{
   uint16_t position = qtr.readLineBlack(sensorValues);
   int error = position - 1000;
  // Sterowanie silnikami w zależności od pozycji linii
  int baseSpeed = 150;   // Podstawowa prędkość silników
  int turnSpeed = 50;    // Korekta prędkości dla zakrętów

  if (error < -500) {
    // Skręć w lewo
    leftMotor(baseSpeed - turnSpeed, true);
    rightMotor(baseSpeed + turnSpeed, true);
  } else if (error > 500) {
    // Skręć w prawo
    leftMotor(baseSpeed + turnSpeed, true);
    rightMotor(baseSpeed - turnSpeed, true);
  } else {
    // Jedź prosto
    leftMotor(baseSpeed, true);
    rightMotor(baseSpeed, true);
  }

}
// Krótki ruch
void demoMode(){

  Serial.println("Tryb DEMO - ruch losowy");
  leftMotor(random(100, 150), true);
  rightMotor(random(100, 150), true);
  delay(500);
  leftMotor(0, true);
  rightMotor(0, true);
  delay(200);
}

void loop() 
{
  void demoMode();
}
//////////////////////////////////////////////
//     .-""""-.        .-""""-.         //////
//    /        \      /        \        //////  
//   /_        _\    /_        _\       //////
//  // \      / \\  // \      / \\      //////
//  |\__\    /__/|  |\__\    /__/|      //////  
//   \    ||    /    \    ||    /       //////
//    \        /      \        /        //////  
//     \  __  /        \  __  /         //////  
//      '.__.'          '.__.'          //////
//       |  |            |  |           //////  
///      |  |            |  |           //////
//////////////////////////////////////////////