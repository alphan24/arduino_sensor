#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <VL53L0X.h>
#include <DS3231.h>

typedef struct { 
    float X, Y, Z; 
}Vector; 

Vector gForceVector;
Vector gForceVector1;
Vector gForceVector2;
Vector k;
Vector gForceVectorAverage;
Vector tempVector;

VL53L0X sensor1,sensor2,sensor3;// define objects for sensors

DS3231  rtc(SDA, SCL);

const int MPU2 = 0x69, MPU1=0x68,chipSelect = 4;
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
int angle;
String dataString = "";
int distance1,distance2,distance3;
File dataFile;

Vector GetMpuValue(const int MPU);
Vector avaregeVectors(Vector vectorA,Vector vectorB);
float dotProduct(Vector vectorA,Vector vectorB);
float magnitude(Vector vector);

void setup(){
  k.X=0;
  k.Y=0;
  k.Z=1;
  mpuInit(MPU1);
  mpuInit(MPU2);
  rtc.begin();
  sensorInit();
  Serial.begin(38400);  
}

void loop(){
  // read three distance sensors:
  distance1 = sensor1.readRangeContinuousMillimeters();// get distance for sensor 1
  distance2 = sensor2.readRangeContinuousMillimeters();// get distance for sensor 2
  distance3 = sensor3.readRangeContinuousMillimeters();// get distance for sensor 3
  
  // read acceleration sensors:
  gForceVector1 = GetMpuValue(MPU1);
  gForceVector2 = GetMpuValue(MPU2);
  gForceVectorAverage = avaregeVectors(gForceVector1,gForceVector2); 
  angle = degrees(acos(dotProduct(gForceVectorAverage,k)/magnitude(gForceVectorAverage)));
  
  // append sesnsor values to stirng:
  dataString = String(distance1)+","+String(distance2)+","+String(distance3)+","+String(angle)+","+String(rtc.getDateStr())+","+String(rtc.getTimeStr());

  dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
    delay(1000);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
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
