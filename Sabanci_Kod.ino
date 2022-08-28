#include <Wire.h>
#include <Servo.h>

//REV
int green, blue, red;
byte busStatus;


//Line counter sensor
int line_sensor_pin = 2;
int lineCounter = 0;
byte colorStatus; // 0 for black and 1 for white
byte prevColorStatus;

//Sensor adresses
int sensorAddress = 0x52;
int mainRegister = 0x00;
int greenRegister = 0x0E;
int blueRegister = 0x11;
int redRegister = 0x14;

int init_last = LOW;
int init_state;
int init_pin = 7;

int victor1_pin = 5;
int victor2_pin = 6;

//Victors
Servo victor1;
Servo victor2;

//Speeds
int max_forward =  96;
int max_reverse = 0;
int motor_stop = 92;

//Color sensor
byte cs;

//PID
double kP = 0.20, kI = 0.06, kD = 0.06;
double setpoint = 399, input, output;
double errorSum, lastError;
unsigned long lastTime;
byte robotState;

//Read register
byte readRegister(int deviceAddress, int registerAddress) {
  byte registerData;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1);
  registerData = Wire.read(); 
  return registerData;
}

//Write register
byte writeRegister(int deviceAddress, int registerAddress, int newRegisterByte)  {
  byte result;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);  
  Wire.write(newRegisterByte); 
  result = Wire.endTransmission();
  delay(5);
  if(result > 0)  
  { 
    Serial.print("ERROR in I2C register writ. Error code: ");
    Serial.println(result); 
  }
  return result;
} 

//PID
void PID() {
  //unsigned long now = millis();
  double timeChange = (double) 0.05;  //(double)(now - lastTime);
  
  input = lineCounter;
  double error = setpoint - input;
  errorSum += (error * timeChange);
  double derivError = (error - lastError) / timeChange;

  output = (kP * error + kI * errorSum + kD * derivError) + motor_stop;
  lastError = error;
  //lastTime = now; 
}



int c = 0;
//Auto period
void autonomousPeriod() {
    int x = 0;
    while (true) {
      if(x == 1000){
        victor1.write(91);
        victor2.write(91);
        robotState = 0;
         return;
      }
      if(x%4 == 0 && x < 105) max_forward++;
      victor1.write(max_forward);
      victor2.write(max_forward);
      x += 1;
      Serial.println(x);
      delay(50);
    }
}


void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  busStatus = writeRegister(sensorAddress, mainRegister, 6);
  pinMode(init_pin, INPUT_PULLUP);
  robotState = 1;
  cs = 0;
  victor1.attach(victor1_pin);
  victor2.attach(victor2_pin);
  victor1.write(91);
  victor2.write(91);
  c = 0;
}

void loop() {
    green = readRegister(sensorAddress, greenRegister);
      blue = readRegister(sensorAddress, blueRegister);
      red = readRegister(sensorAddress, redRegister);
      Serial.print("Green:");
      Serial.println(green);
      Serial.print("Blue: ");
      Serial.println(blue);
      Serial.print("Red: ");
      Serial.println(red);
    init_state = HIGH; //digitalRead(init_pin);
    
    if(robotState && init_last == LOW && init_state == HIGH) {
      Serial.println("Autonomous period has started");
      autonomousPeriod();
      if(!robotState) {
        Serial.println("Robot has successfully stopped!");
      }
    }

    init_last = init_state;
    delay(50);
}
