//For the accelerometer
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

int scaleIt(double value){ //This function will scale all values to an integer between 100 and 999.
  int scaledValue=(value*899)+100;
  return scaledValue;
}

//-----Begin Brake Position Sensor-----
#define BRAKE_POSITION_SENSOR_PORT A5
double brakePosition=0;
//-----Begin Brake Position Sensor-----

//-----Begin Steering Angle Sensor-----
#define STEERING_ANGLE_SENSOR_PORT A6
double steeringAngle=0;
//-----End Steering Angle Sensor-----

//-----Begin Suspension Sensor-----
#define SUSPENSION_SENSOR_1_PORT A0
#define SUSPENSION_SENSOR_2_PORT A1
#define SUSPENSION_SENSOR_3_PORT A2
#define SUSPENSION_SENSOR_4_PORT A3

double suspension1=0;
double suspension2=0;
double suspension3=0;
double suspension4=0;
//-----End Suspension Sensor-----

//-----Begin Fuel Pressure Sensor-----
#define PRESSURE_SENSOR_PORT A4
double fuelPressure=0;
//-----End Fuel Pressure Sensor-----

//-----Begin Accelerometer-----
Adafruit_MMA8451 mma = Adafruit_MMA8451(); //The breakout is connected on Pins 20 and 21
double xAccel=0;
double yAccel=0;
double zAccel=0;
double G=78.5; //This is 9.8m/s^2*8G for max out
//-----End Accelerometer-----

void setup()
{
  Serial.begin(9600);
  
  //-----Begin Accelerometer-----
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_8_G);
  //-----End Accelerometer-----
}


void loop()
{
  String outString=""; //This will store the fixed-length values to be sent to LabView
  
  //-----Begin Brake Position Sensor-----
  brakePosition=analogRead(BRAKE_POSITION_SENSOR_PORT);
  brakePosition=(map(brakePosition, 26, 1023, 0, 100))/100.0; //This gives the amount the brake pedal has been depressed from 0% to 100% depression.
  //-----End Brake Position Sensor-----
  
  //-----Begin Steering Angle Sensor-----
  steeringAngle=analogRead(STEERING_ANGLE_SENSOR_PORT);
  steeringAngle=(((steeringAngle*3.05)-1564.37)+201)/399.0; //This gives the angle in degrees with 0 at 12 o'clock
  //-----End Steering Angle Sensor-----
  
  //-----Begin Suspension Sensor-----
  suspension1=analogRead(SUSPENSION_SENSOR_1_PORT);
  suspension1=((((suspension1*0.0028528265)-1.4635)*100)+147)/293.0; //This scales the analog input to distance depressed or expanded in inches with 0 being the middle distance between the max despression and max extension.
  
  suspension2=analogRead(SUSPENSION_SENSOR_2_PORT);
  suspension1=((((suspension2*0.0028528265)-1.4635)*100)+147)/293.0;
  
  suspension3=analogRead(SUSPENSION_SENSOR_3_PORT);
  suspension1=((((suspension3*0.0028528265)-1.4635)*100)+147)/293.0;
  
  suspension4=analogRead(SUSPENSION_SENSOR_4_PORT);
  suspension1=((((suspension4*0.0028528265)-1.4635)*100)+147)/293.0;
  //-----End Suspension Sensor-----
  
  //-----Begin Fuel Pressure Sensor-----
  fuelPressure=analogRead(PRESSURE_SENSOR_PORT);
  fuelPressure=((fuelPressure*0.12)-11.83)/100.0; //This turns the voltage reading to PSI
  //-----End Fuel Pressure Sensor-----
  
  //-----Begin Accelerometer-----
  sensors_event_t event; 
  mma.getEvent(&event);
  
  xAccel=event.acceleration.x;
  xAccel=(xAccel+G)/(2*G);

  yAccel=event.acceleration.y;
  yAccel=(yAccel+G)/(2*G);
  
  zAccel=event.acceleration.z;
  zAccel=(zAccel+G)/(2*G);
  //-----End Accelerometer-----
  
  //-----Begin ECU-----
 
  //-----End ECU-----
    outString=scaleIt(suspension1)+scaleIt(suspension2)+scaleIt(suspension3)+scaleIt(suspension4)+scaleIt(fuelPressure)+scaleIt(brakePosition)+scaleIt(steeringAngle)+scaleIt(xAccel)+scaleIt(yAccel)+scaleIt(zAccel);
    Serial1.println(outString); //Writes the string to TX1, where the XBee will be connected
}
