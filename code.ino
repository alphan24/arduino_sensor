#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DS3231.h>
#include <SPI.h>
#include <SD.h>
#include <VL53L0X.h>

LiquidCrystal_I2C lcd(0x27,20,4);
DS3231  rtc(SDA, SCL);
VL53L0X sensor1,sensor2,sensor3;// define objects for sensors

#define TCAADDR 0x70
const int MPUADDR = 0x69;

typedef struct { 
    float X, Y, Z; 
}Vector; 

typedef struct { 
    int sensor1, sensor2, sensor3; 
}Distance;

Vector K;
Vector gForceVector;
Vector gForceVector1;
Vector gForceVector2;
Vector gForceVectorAverage;
Vector tempVector;

Distance distance, distanceData;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
int angle;
String time;
String dataString1 = "";
String dataString2 = "";
String dataString3 = "";
const int chipSelect = 10;

Vector GetMpuValue(const int MPUADDR);
Vector avaregeVectors(Vector vectorA,Vector vectorB);
float dotProduct(Vector vectorA,Vector vectorB);
float magnitude(Vector vector);

void setup() {
  // put your setup code here, to run once:
 rtc.begin();
 sensorInit();
 Serial.begin(38400); 
 lcd.init();
 lcd.backlight();
 lcd.setCursor(0,0);
 
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    lcd.print("Card failed");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  lcd.print("card initialized.");
  Wire.begin(); 
  
  K.X=1;
  K.Y=0;
  K.Z=0;
  
  Serial.println("init done");
  lcd.clear();
  lcd.print("init done");
}

void loop() {
 time = rtc.getTimeStr(); // get current time
 
 // Get acceleration sensor values
 tcaselect(0);
 gForceVector1 = GetMpuValue(MPUADDR);
 tcaselect(1);
 gForceVector2 = GetMpuValue(MPUADDR);

 //calculate angle
 gForceVector = avaregeVectors(gForceVector1,gForceVector2);
 angle = degrees(acos(dotProduct(gForceVector,K)/magnitude(gForceVector)));

 distanceData = getDistance();//read distance sensor values
 
 Serial.println();
 Serial.println(time);

 Serial.print("angle: ");
 Serial.println(angle);
 
 Serial.println("MPU1");
 Serial.print("X: ");
 Serial.print(" Y: ");
 Serial.print(gForceVector1.Y);
 Serial.print(" Z: ");
 Serial.print(gForceVector1.Z);
 Serial.println();
 
 Serial.println("MPU2");
 Serial.print("X: ");
 Serial.print(gForceVector2.X);
 Serial.print(" Y: ");
 Serial.print(gForceVector2.Y);
 Serial.print(" Z: ");
 Serial.print(gForceVector2.Z);
 Serial.println();

 Serial.print("Distance 1: ");
 Serial.print(distanceData.sensor1);
 Serial.print(" Distance 2: ");
 Serial.print(distanceData.sensor2);
 Serial.print(" Distance 2: ");
 Serial.print(distanceData.sensor3);
 Serial.println();

 lcd.clear();
 lcd.setCursor(12,0);
 lcd.print(time);
 lcd.setCursor(0,0);
 lcd.print("Angle: ");
 lcd.print(angle);
 lcd.setCursor(0,1);
 lcd.print(distanceData.sensor1);
 lcd.setCursor(5,1);
 lcd.print(distanceData.sensor2);
 lcd.setCursor(10,1);
 lcd.print(distanceData.sensor3);
 lcd.setCursor(0,2);
 lcd.print(gForceVector1.X);
 lcd.setCursor(6,2);
 lcd.print(gForceVector1.Y);
 lcd.setCursor(12,2);
 lcd.print(gForceVector1.Z);
 lcd.setCursor(0,3);
 lcd.print(gForceVector2.X);
 lcd.setCursor(6,3);
 lcd.print(gForceVector2.Y);
 lcd.setCursor(12,3);
 lcd.print(gForceVector2.Z);
 

  // append sesnsor values to stirng:
  dataString1 = String(gForceVector1.X)+","+String(gForceVector1.Y)+","+String(gForceVector1.Z)+",";
  dataString2 = String(gForceVector2.X)+","+String(gForceVector2.Y)+","+String(gForceVector2.Z)+",";
  dataString3 = String(distanceData.sensor1)+","+String(distanceData.sensor2)+","+String(distanceData.sensor3)+","
  +String(angle)+","+time;
  
  // write collected data to SD card 
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(dataString1);
    dataFile.print(dataString2);
    dataFile.println(dataString3);
    dataFile.close();
    Serial.println(dataString1);
    Serial.println(dataString2);
    Serial.println(dataString3);
    delay(100);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
    lcd.setCursor(0,0);
    lcd.print("file error");
  }
  delay(1000);
}
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
Vector avaregeVectors(Vector vectorA,Vector vectorB){
  tempVector.X = (vectorA.X+vectorB.X)/2;
  tempVector.Y = (vectorA.Y+vectorB.Y)/2;
  tempVector.Z = (vectorA.Z+vectorB.Z)/2;
  return tempVector;
}
void mpuInit(const int MPUADDR){
  Wire.begin();
  Wire.beginTransmission(MPUADDR);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();   
  Wire.beginTransmission(MPUADDR);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission(); 
}
Vector GetMpuValue(const int MPUADDR){
  Wire.beginTransmission(MPUADDR); 
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPUADDR,6);
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
Distance getDistance(){
  distance.sensor1 = sensor1.readRangeSingleMillimeters();// get distance for sensor 1
  distance.sensor2 = sensor2.readRangeSingleMillimeters();// get distance for sensor 2
  distance.sensor3 = sensor3.readRangeSingleMillimeters();// get distance for sensor 3
  return distance;
}
void sensorInit(){
  
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  Wire.begin();

  //SENSOR
  digitalWrite(5, HIGH);
  sensor1.init(true);
  sensor1.setAddress(0x30);
  Serial.println("set address 0x30 for first sensor");
  delay(100);

  digitalWrite(6, HIGH);
  sensor2.init(true);
  sensor2.setAddress(0x31);
  Serial.println("set address 0x31 for second sensor");
  delay(100);

  digitalWrite(7, HIGH);
  sensor3.init(true);
  sensor3.setAddress(0x32);
  Serial.println("set address 0x32 for third sensor");
  delay(100);

  sensor1.setTimeout(500);
  sensor2.setTimeout(500);
  sensor3.setTimeout(500);
  
  tcaselect(0);
  mpuInit(MPUADDR);
  tcaselect(1);
  mpuInit(MPUADDR);
}
