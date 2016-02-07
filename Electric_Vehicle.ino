//The Wire library is used for I2C communication
#include <Wire.h>
#include <Servo.h>
#include <SFE_MMA8452Q.h>

MMA8452Q accel;

int runTime = 20000; // approximately 1000 for each foot
float goalDistance = 2.159; // meters
float conversionFactor = 5.01; // bigger makes it travel less
int loopTime = 50;//50;
float distance = 0.0;
float velocity = 0.0;
float old_velocity = 0.0;
float old_acceleration = 0.0;
float kp = 0.9;
float ki = 0.0003;
float kd = 0.05;
float leftCorrection = 0;//0.5;
float integral = 0.0;

float averageXAccel = 0.0;
float accelerationX = 0.0;

Servo left;
Servo right;
bool readyToRun = false;
bool justClicked = false;
int standardSpeed = 15;
int turnThreshold = 10;
const int buttonPin = 2; 
float angle = 0;
// stuff i have no idea about
char WHO_AM_I_GYRO = 0x00;
char SMPLRT_DIV= 0x15;
char DLPF_FS = 0x16;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;
char DLPF_CFG_0 = 1<<0;
char DLPF_CFG_1 = 1<<1;
char DLPF_CFG_2 = 1<<2;
char DLPF_FS_SEL_0 = 1<<3;
char DLPF_FS_SEL_1 = 1<<4;
char itgAddress = 0x69;

//In the setup section of the sketch the serial port will be configured, the i2c communication will be initialized, and the itg-3200 will be configured.
void setup()
{
  //Create a serial connection using a 9600bps baud rate.
  Serial.begin(9600);
  /* ACCEL INITIATION */
  accel.init();
  /* GYRO INITIATION */
  //Initialize the I2C communication. This will set the Arduino up as the 'Master' device.
  //Wire.begin();
  //Read the WHO_AM_I register and print the result
  char id=0; 
  id = itgRead(itgAddress, WHO_AM_I_GYRO);  
  Serial.print("ID: ");
  Serial.println(id, HEX);
  //Configure the gyroscope
  //Set the gyroscope scale for the outputs to +/-2000 degrees per second
  itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
  //Set the sample rate to 100 hz
  itgWrite(itgAddress, SMPLRT_DIV, 9);

 
  left.attach(10);
  left.write(90);  // set servo to mid-point
  right.attach(11);
  right.write(90);
  pinMode(buttonPin, INPUT);
  readyToRun = true;
}

//The loop section of the sketch will read the X,Y and Z output rates from the gyroscope and output them in the Serial Terminal
void loop()
{
   fixAverageAccel();
  if(digitalRead(buttonPin) == HIGH) {
    if(!justClicked && readyToRun) {
      justClicked = true;
      readyToRun = false;
      angle = 0;
      integral = 0;
       // for(int i = 0; i < runTime/loopTime; i++)
      while(distance < goalDistance)
      {
        float zRate;
        float oldAngle = 0;
        zRate = readZ();
        //Serial.println(zRate);

        angle += zRate / (1000 / loopTime);  // Divide the degrees per second by the amount of times looping each second
        integral = integral + angle * loopTime;
        if(abs(integral) < 50)
          integral = angle;
        if((angle > 0) && (integral < 0))
          integral = 0;
        if((angle < 0) && (integral > 0))
          integral = angle;

        float derivative = (angle - oldAngle) / loopTime;

        int correction = (kp * angle) + (ki * integral) + (kd * derivative);

        int leftSpeed = 0 + standardSpeed - correction;

        int rightSpeed = 180 - standardSpeed - correction;
       
        if(leftSpeed > 90)
          leftSpeed = 90;
        if(rightSpeed < 90)
          rightSpeed = 90;
        /*Serial.print("left: "); 
        Serial.println(leftSpeed);
        Serial.print("right: ");
        Serial.println(rightSpeed);
        Serial.print("angle: ");
        Serial.println(angle);*/
        
        left.write(leftSpeed);
        right.write(rightSpeed);

        
        accel.read();
        accelerationX = (accel.cx - averageXAccel) * conversionFactor;
        /*if(accelerationX < 0.005)
          accelerationX = 0.0;*/
        Serial.print("Acceleration: ");
        Serial.print(accelerationX);
        Serial.println( "m/s^2");
        Serial.print("Velocity: ");
        velocity = old_velocity + ((accelerationX + old_acceleration)/2.0 * loopTime/1000);
        Serial.print(velocity);
        Serial.println(" m/s");
        distance = distance + ((velocity + old_velocity)/2.0 * loopTime/1000);
        Serial.print("distance traveled: ");
        Serial.print(distance);
        Serial.println(" m");
        old_velocity = velocity;
        old_acceleration = accelerationX;
        oldAngle = angle;
      
        //Wait ms before reading the values again. (Remember, the output rate was set to 100hz and 1reading per 10ms = 100hz.)
        delay(loopTime);
      }
      left.write(90);
      right.write(90);
      readyToRun = true;
    }
  } else {
    justClicked = false;
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  }
  
}

void printCalculatedAccels()
{ 
  Serial.print(accel.cx, 3);
  Serial.print("\t");
  Serial.print(accel.cy, 3);
  Serial.print("\t");
  Serial.print(accel.cz, 3);
  Serial.print("\t");
  Serial.println();
}


void fixAverageAccel()
{
  float sum = 0.0;
  for(int i = 0; i < 200; i++)
  {
    if(accel.available())
      accel.read();
    sum += accel.cx;
    delay(10);
  }

  averageXAccel = 0;//(sum / 200.0) * 1.04;
}


void itgWrite(char address, char registerAddress, char data)
{
  //Initiate a communication sequence with the desired i2c device
  Wire.beginTransmission(address);
  //Tell the I2C address which register we are writing to
  Wire.write(registerAddress);
  //Send the value to write to the specified register
  Wire.write(data);
  //End the communication sequence
  Wire.endTransmission();
}
unsigned char itgRead(char address, char registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;
  //Send the register address to be read.
  Wire.beginTransmission(address);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();
  //Ask the I2C device for data
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  //End the communication sequence.
  Wire.endTransmission();
  //Return the data read during the operation
  return data;
}
//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int zRate = readZ();
float readZ(void)
{
  int data=0;
  data = itgRead(itgAddress, GYRO_ZOUT_H)<<8;
  data |= itgRead(itgAddress, GYRO_ZOUT_L);

  return data;
}
