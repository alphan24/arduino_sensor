#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <VL53L0X.h>
#include <DS3231.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

typedef struct { 
    float X, Y, Z; 
}Vector; 

Vector gForceVector;
Vector gForceVector1;
Vector gForceVector2;
Vector k;
Vector gForceVectorAverage;
Vector tempVector;

typedef struct { 
    int sensor1, sensor2, sensor3; 
}Distance;

Distance distance, distanceData;
VL53L0X sensor1,sensor2,sensor3;// define objects for sensors

DS3231  rtc(SDA, SCL);

const int MPU = 0x69;
const int chipSelect = 4;
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
int angle;
String dataString = "";

//Define variables
#define I2C_ADDR 0x27 //Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

LiquidCrystal_I2C lcd(I2C_ADDR, En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);//Initialise the LCD
Vector GetMpuValue(const int MPU);
Vector avaregeVectors(Vector vectorA,Vector vectorB);
Distance getDistance();
float dotProduct(Vector vectorA,Vector vectorB);
float magnitude(Vector vector);
String time;
void setup(){
  k.X=1;
  k.Y=0;
  k.Z=0;
  
  sensorInit();
  lcd.begin (16,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  rtc.begin();
  Serial.begin(38400);
  pinMode(10, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
   Wire.begin(); 
}

void loop(){
  time = rtc.getTimeStr();
  distanceData = getDistance();//read distance sensor values
  
  // read acceleration sensors:
  gForceVector1 = GetMpuValue(MPU);
  angle = degrees(acos(dotProduct(gForceVector1,k)/magnitude(gForceVector1)));
  
  // append sesnsor values to stirng:
  dataString = String(distanceData.sensor1)+","+String(distanceData.sensor2)+","+String(distanceData.sensor3)+","
  +String(angle)+","+time;
  
  lcd.clear ( );
  lcd.setCursor(0, 0);
  lcd.print(String(distanceData.sensor1)+" "+String(distanceData.sensor2)+" "+String(distanceData.sensor3));
  lcd.setCursor(0, 1);
  lcd.print(String(angle));
  lcd.setCursor(4, 1);
  lcd.print(time);
  Serial.print(String(gForceVector1.X)+" "+String(gForceVector1.Y)+" "+String(gForceVector1.Z));
  Serial.println(dataString);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
    delay(100);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  delay(100);
}
Distance getDistance(){
  distance.sensor1 = sensor1.readRangeContinuousMillimeters();// get distance for sensor 1
  distance.sensor2 = sensor2.readRangeContinuousMillimeters();// get distance for sensor 2
  distance.sensor3 = sensor3.readRangeContinuousMillimeters();// get distance for sensor 3
  return distance;
}
void sensorInit(){
  Wire.begin();

  sensor1.init();// initialize sensor 1
  sensor1.setTimeout(500);// set time out for sensor 1
 
  sensor2.init();// initialize sensor 2
  sensor2.setTimeout(500);// set time out for sensor 2
 
  sensor3.init();// initialize sensor 3
  sensor3.setTimeout(500);// set time out for sensor 3

  sensor1.startContinuous();// measure continuously for sensor 1
  sensor2.startContinuous();// measure continuously for sensor 2  
  sensor3.startContinuous();// measure continuously for sensor 3
  
  mpuInit(MPU);
}
Vector avaregeVectors(Vector vectorA,Vector vectorB){
  tempVector.X = (vectorA.X+vectorB.X)/2;
  tempVector.Y = (vectorA.Y+vectorB.Y)/2;
  tempVector.Z = (vectorA.Z+vectorB.Z)/2;
  return tempVector;
}
void mpuInit(const int MPU){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();  
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission(); 
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission(); 
}
Vector GetMpuValue(const int MPU){
  Wire.beginTransmission(MPU); 
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU,6);
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); 
  accelY = Wire.read()<<8|Wire.read(); 
  accelZ = Wire.read()<<8|Wire.read();
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
  gForceVector.X = gForceX;
  gForceVector.Y = gForceY;
  gForceVector.Z = gForceZ;
  delay(10);
  return gForceVector;
}
float dotProduct(Vector vectorA,Vector vectorB){
 return vectorA.X*vectorB.X+vectorA.Y*vectorB.Y+vectorA.Z*vectorB.Z;
}
float magnitude(Vector vector){
 return sqrt(sq(vector.X)+sq(vector.Y)+sq(vector.Z));
}
