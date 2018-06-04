#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TimerOne.h>
#include <dht.h>

dht DHT;

SoftwareSerial ss(2, 3); // Used by GPS

File GPS; //File to write NEMA data
File GYRO; //File to write gyro data
File temp_file; //File to write temperature and humidity data

//Pins used
int SD_CS = 4;
int SD_MOSI = 11;
int SD_MISO = 12;
int SD_SCK = 13;
int GPS_RX = 3;
int GPS_TX = 2;
int GYRO_SDA = A4;
int GYRO_SCL = A5;
int GAS = 9;
int DHT_PIN = 8;


//Used by MPU9250
#define  MPU9250_ADDRESS           0x68
#define  MAG_ADDRESS               0x0C

#define  GYRO_FULL_SCALE_250_DPS   0x00
#define  GYRO_FULL_SCALE_500_DPS   0x08
#define  GYRO_FULL_SCALE_1000_DPS  0x10
#define  GYRO_FULL_SCALE_2000_DPS  0x18

#define  ACC_FULL_SCALE_2_G        0x00
#define  ACC_FULL_SCALE_4_G        0x08
#define  ACC_FULL_SCALE_8_G        0x10
#define  ACC_FULL_SCALE_16_G       0x18

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void gyro_settings()
{
   // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

// Initial time
long int ti;
volatile bool intFlag=false;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  // GPS Setup
  ss.begin(9600);
  delay(1000);
  Serial.println("GPS Setup Complete!");

  //SD Card Setup
  pinMode(10, OUTPUT); //Reserved for SD.h library
  SD.begin(SD_CS);

  // Gyro setup
  Wire.begin();
  gyro_settings();
  ti=millis(); // Store initial time

  // File setup
  if (SD.exists("GPS.ubx")) {
    SD.remove("GPS.ubx");
  }
  if (SD.exists("GYRO.txt")) {
    SD.remove("GYRO.txt");
  }

  // MQ-3 and DHT11 pin initialization
  pinMode(GAS, INPUT);
  pinMode(TEMP, INPUT);
}

// Counter Used by MPU9250
long int cpt=0;
void callback()
{ 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}


void loop() {
  if(digitalRead(GAS)) while(1);
  // Create and append GPS data to a file
  GPS = SD.open("GPS.ubx", FILE_WRITE);
  // The .ubx file needs to be opened in u-center and exported as .kml file to be read from Google Earth.
  if(GPS){  
    while (ss.available()) {
      GPS.write(ss.read());
    }
    GPS.close();
  }
  tempRead();
  // Create and append Gyro data to a file
  gyro();
}


void gyro()
{
  GYRO = SD.open("GYRO.txt", FILE_WRITE);
  while (!intFlag);
  intFlag=false;
  
  // Display time
  GYRO.print (millis()-ti,DEC);
  GYRO.print ("\t");  
  // Display data counter
  GYRO.print (cpt++,DEC);
  GYRO.print ("\t");

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data
  
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];
  
  // Display values
  // Accelerometer
  GYRO.print (ax,DEC); 
  GYRO.print ("\t");
  GYRO.print (ay,DEC);
  GYRO.print ("\t");
  GYRO.print (az,DEC);  
  GYRO.print ("\t");
  
  // Gyroscope
  GYRO.print (gx,DEC); 
  GYRO.print ("\t");
  GYRO.print (gy,DEC);
  GYRO.print ("\t");
  GYRO.print (gz,DEC);  
  GYRO.print ("\t");

  // :::  Magnetometer ::: 
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  
  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=-(Mag[3]<<8 | Mag[2]);
  int16_t my=-(Mag[1]<<8 | Mag[0]);
  int16_t mz=-(Mag[5]<<8 | Mag[4]);
    
  // Magnetometer
  GYRO.print (mx+200,DEC); 
  GYRO.print ("\t");
  GYRO.print (my-70,DEC);
  GYRO.print ("\t");
  GYRO.print (mz-700,DEC);  
  GYRO.print ("\t");
  
  GYRO.println("");
  GYRO.close();
  //delay(100);
}

void tempRead()
{
  int chk = DHT.read(DHT_PIN);
  temp_file = SD.open("Temp.txt", FILE_WRITE);
  temp_file.print(millis()-ti,DEC);
  temp_file.print("\t"); 
  temp_file.print("Temperature: ");
  temp_file.print(DHT.tempetarute);
  temp_file.print("C");
  temp_file.print("\t");
  temp_file.print("Humidity: ");
  temp_file.print(DHT.humidity);
  temp_file.println("%");
  temp_file.close();
  //delay(100);
}
// Amin!
