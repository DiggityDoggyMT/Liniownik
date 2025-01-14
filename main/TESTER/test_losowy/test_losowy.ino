
#include <QTRSensors.h>




// definiowanie pinow
 const int MOTOR_A_IN1 = A1  // Prawy silnik 
 const int MOTOR_A_IN2 = A2  // Prawy silnik 
 const int MOTOR_B_IN1 = A3  // Lewy silnik 
 const int MOTOR_B_IN2 = A4  // Lewy silnik 

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
    analogWrite(A3, 0); //Silnik A - obroty w lewo
    analogWrite(A4, speed); 
  } else {
    analogWrite(A3, speed); //Silnik A - obroty w prawo
    analogWrite(A4, 0); 
  }
}

// PRAWY SILNIK
void rightMotor(int speed, bool forward) {
  if (forward) {
    analogWrite(A1, 0); //Silnik B - obroty w lewo
    analogWrite(A2, speed); 
  } else {
    analogWrite(A1, speed); //Silnik B - obroty w prawo
    analogWrite(A2, 0); 
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