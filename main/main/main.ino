
#include <QTRSensors.h>


void emittersOn()
sensors.read(sensor_values);
 QTRSensorsRC qtrrc((unsigned char[]) {6, 7, 8, 9, 10, 11, 12, 13}, 8); // 8 czujników
 int sensorValues[8]; // Tablica odczytów czujników

// definiowanie pinow
#define MOTOR_A_IN1 A1  // Lewy silnik 
#define MOTOR_A_IN2 A2  // Lewy silnik 
#define MOTOR_B_IN1 A3  // Prawy silnik 
#define MOTOR_B_IN2 A4  // Prawy silnik 
#define BUTTON_PIN  A5 // PRZYCISK
#define TIME_LIMIT 10000  

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
 
   int i;
  for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtrrc.calibrate();
    delay(20);
  }
  }


// Wciśnięcie przycisku
void TRYB_ROBOTA() {
   

  // Sprawdzamy, czy przycisk został wciśnięty (zbocze opadające)
  if (buttonState == LOW && lastButtonState == HIGH) {
    clickCount++; // Zwiększamy licznik kliknięć
  }

  lastButtonState = buttonState; // Aktualizacja poprzedniego stanu przycisku
 if (millis() - startTime >= TIME_LIMIT) {
    // Sprawdzenie ilości kliknięć i wybór trybu
    if (clickCount >= 2 ){  // Przykład: 5 lub więcej kliknięć = tryb NORMALNY
      mode = NORMAL_MODE;
    } else if{
      mode = DEMO_MODE;
    }
    clickCount = 0;            // Zerowanie licznika kliknięć
    startTime = millis();      // Restart czasu pomiaru

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
    analogWrite(3, 255); 
  } else {
    analogWrite(2, 255); //Silnik A - obroty w prawo
    analogWrite(3, 0); 
  }
}

// PRAWY SILNIK
void rightMotor(int speed, bool forward) {
  if (forward) {
    analogWrite(4, 0); //Silnik B - obroty w lewo
    analogWrite(5, 255); 
  } else {
    analogWrite(4, 255); //Silnik B - obroty w prawo
    analogWrite(5, 0); 
  }
}

//TRYB LINE FOLLOWER
void lineFollower()
{
   int position = qtrrc.readLine(sensors);
   int error = position - 1000;
  // Sterowanie silnikami w zależności od pozycji linii
  int baseSpeed = 150;   // Podstawowa prędkość silników
  int turnSpeed = 50;    // Korekta prędkości dla zakrętów

  if (error < -500) {
    // Skręć w lewo
    leftMotor(baseSpeed - turnSpeed, true);
    rightMotor(baseSpeed + turnSpeed, true);
  } else if (position > 500) {
    // Skręć w prawo
    leftMotor(baseSpeed + turnSpeed, true);
    rightMotor(baseSpeed - turnSpeed, true);
  } else {
    // Jedź prosto
    leftMotor(baseSpeed, true);
    rightMotor(baseSpeed, true);
  }

}

void demoMode() {
  Serial.println("Tryb DEMO - ruch losowy");

  leftMotor(random(100, 150), true);
  rightMotor(random(100, 150), true);
  delay(500); // Krótki ruch

  leftMotor(0, true);
  rightMotor(0, true);
  delay(200);
}

void loop() 
{
  TRYB_ROBOTA();
 
  if (wybrany_tryb == DEMO_MODE) {
    demoMode(); // Tryb demo
  } else if (wybrany_tryb == NORMAL_MODE) {
    lineFollower(); // Tryb normalny - śledzenie linii
  }

}
//////////////////////////////
///////////////
///////////////
/////////////////////////////
///////////////
///////////////
/////////////////////////////
///////////////
///////////////
/////////////////////////////
///////////////
///////////////
//////////////
///////////////
///////////////
//////////////
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
///////////////////////////////////////////////////////
//////////////////////////////////////////WSZYSTKO CO PONIZEJ TO KLAMSTWO I NIEPRAWDA/////////////////////////////
// do testowania silników
void demoMode() {
  Serial.println("Tryb DEMO - ruch losowy");

  leftMotor(random(100, 150), true);
  rightMotor(random(100, 150), true);
  delay(500); // Krótki ruch

  leftMotor(0, true);
  rightMotor(0, true);
  delay(200);
}
void setup()
{

  
  // optional: wait for some input from the user, such as  a button press
 
  // then start calibration phase and move the sensors over both
  // reflectance extremes they will encounter in your application:
  int i;
  for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
 
  // optional: signal that the calibration phase is now over and wait for further
  // input from the user, such as a button press
}

void loop()
{
  unsigned int sensors[3];
  // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 2000, with 1000 corresponding to the line
  // over the middle sensor.
  int position = qtr.readLine(sensors);
 
  // if all three sensors see very low reflectance, take some appropriate action for this 
  // situation.
  if (sensors[0] > 750 && sensors[1] > 750 && sensors[2] > 750)
  {
    // do something.  Maybe this means we're at the edge of a course or about to fall off 
    // a table, in which case, we might want to stop moving, back up, and turn around.
    return;
  }
 
  // compute our "error" from the line position.  We will make it so that the error is
  // zero when the middle sensor is over the line, because this is our goal.  Error
  // will range from -1000 to +1000.  If we have sensor 0 on the left and sensor 2 on
  // the right,  a reading of -1000 means that we see the line on the left and a reading
  // of +1000 means we see the line on the right.
  int error = position - 1000;
 
  int leftMotorSpeed = 100;
  int rightMotorSpeed = 100;
  if (error < -500)  // the line is on the left
    leftMotorSpeed = 0;  // turn left
  if (error > 500)  // the line is on the right
    rightMotorSpeed = 0;  // turn right
 
  // set motor speeds using the two motor speed variables above
}

int lastError = 0;
 
void loop()
{
  unsigned int sensors[3];
  // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 2000, with 1000 corresponding to the line over
  // the middle sensor
  int position = qtr.readLine(sensors);
 
  // compute our "error" from the line position.  We will make it so that the error is zero
  // when the middle sensor is over the line, because this is our goal.  Error will range
  // from -1000 to +1000.  If we have sensor 0 on the left and sensor 2 on the right,  
  // a reading of -1000 means that we see the line on the left and a reading of +1000 
  // means we see the line on the right.
  int error = position - 1000;
 
  // set the motor speed based on proportional and derivative PID terms
  // KP is the a floating-point proportional constant (maybe start with a value around 0.1)
  // KD is the floating-point derivative constant (maybe start with a value around 5)
  // note that when doing PID, it's very important you get your signs right, or else the
  // control loop will be unstable
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
 
  // M1 and M2 are base motor speeds.  That is to say, they are the speeds the motors
  // should spin at if you are perfectly on the line with no error.  If your motors are
  // well matched, M1 and M2 will be equal.  When you start testing your PID loop, it
  // might help to start with small values for M1 and M2.  You can then increase the speed
  // as you fine-tune your PID constants KP and KD.
  int m1Speed = M1 + motorSpeed;
  int m2Speed = M2 - motorSpeed;
 
  // it might help to keep the speeds positive (this is optional)
  // note that you might want to add a similiar line to keep the speeds from exceeding
  // any maximum allowed value
  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
 
  // set motor speeds using the two motor speed variables above
}


  POWROT=0;
  czas=millis();
  while(POWROT==0 && czas<10000)
  {
    tryb=digitalRead(BUTTON_PIN);
    delay(10000)
      if (tryb>1)
        {
          wybrany_tryb=NORMAL_MODE;
          POWROT=1;
        }
      else if (tryb=1)
      {
        wybrany_tryb=DEMO_MODE;
          POWROT=1;
      }
      else
      {
        tryb=0;
        czas=0;
      }
  }
