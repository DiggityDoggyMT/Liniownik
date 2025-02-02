
#include <QTRSensors.h>


void emittersOn()
unsigned int sensor_values[8];
sensors.read(sensor_values);

#define MOTOR_A_IN1 2  // Lewy silnik 
#define MOTOR_A_IN2 3  // Lewy silnik 
#define MOTOR_B_IN1 4   // Prawy silnik 
#define MOTOR_B_IN2 5   // Prawy silnik 
const int buttonPin = A3;


 QTRSensorsRC qtrrc((unsigned char[]) {6, 7, 8, 9, 10, 11, 12, 13}, 8); // 8 czujników

 int sensorValues[8]; // Tablica odczytów czujników
// create an object for your type of sensor (RC or Analog)
// in this example we have three sensors on analog inputs 0 - 2 (digital pins 14 - 16)

// Kalibracja QTR-8RC
  Serial.begin(9600);
  Serial.println("Kalibracja czujników...");
  for (int i = 0; i < 400; i++) { // Obracanie robota w celu kalibracji
    qtrrc.calibrate();
    delay(10);
  }
  Serial.println("Kalibracja zakończona!");


 void setup()
{

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(buttonPin, INPUT);   

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {   
    // turn LED on:   
    digitalWrite(ledPin, HIGH); 
  }
  else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
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
    digitalWrite(2, LOW); //Silnik A - obroty w lewo
    digitalWrite(3, HIGH); 
  } else {
    digitalWrite(2, HIGH); //Silnik A - obroty w prawo
    digitalWrite(3, LOW); 
  }
}
// PRAWY SILNIK
void rightMotor(int speed, bool forward) {
  if (forward) {
    digitalWrite(4, LOW); //Silnik B - obroty w lewo
    digitalWrite(5, HIGH); 
  } else {
    digitalWrite(4, HIGH); //Silnik B - obroty w prawo
    digitalWrite(5, LOW); 
  }
}





void loop() 
{
   int error = position - 1000;
  // Sterowanie silnikami w zależności od pozycji linii
  int baseSpeed = 150;   // Podstawowa prędkość silników
  int turnSpeed = 50;    // Korekta prędkości dla zakrętów

  if (position < 2000) {
    // Skręć w lewo
    leftMotor(baseSpeed - turnSpeed, true);
    rightMotor(baseSpeed + turnSpeed, true);
  } else if (position > 4000) {
    // Skręć w prawo
    leftMotor(baseSpeed + turnSpeed, true);
    rightMotor(baseSpeed - turnSpeed, true);
  } else {
    // Jedź prosto
    leftMotor(baseSpeed, true);
    rightMotor(baseSpeed, true);
  }


}

//////////////////////////////////////////

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
