    /*
  MAX30102- Blood Oxyge -Module
  https://electropeak.com/learn/
  Based on Arduino Library Example
*/


#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614(0x5A);

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

uint8_t accel_sensor_value_x = 0;
uint8_t accel_sensor_value_y = 0;
uint8_t accel_sensor_value_z = 0;

bool first = true;

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  if (mlx.begin()) {
     Serial.println("mlx found!");  
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop()
{
  accel_sensor_value_x = analogRead(34); // for universal, x is 32
  accel_sensor_value_y = analogRead(32); // for universal, y is 35
  accel_sensor_value_z = analogRead(35); //for universal, z is 34
  Serial.println(accel_sensor_value_x);
  Serial.println(accel_sensor_value_y);
  Serial.println(accel_sensor_value_z);
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  if (first) {
    //read the first 100 samples, and determine the signal range
    first = false;
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
  
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
  
      //Serial.print(F("red="));
      //Serial.print(redBuffer[i], DEC);
      //Serial.print(F(", ir="));
      //Serial.println(irBuffer[i], DEC);
    }
  
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate); 
  }

  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    if (i % 5 == 0) {
      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);
  
      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);
  
      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);
  
      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
  
      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC); 
    }
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  String line1, line2;
  line1 = String(mlx.readObjectTempC(), 1);
  line1 += "[C]";
  line2 = "Ambient: ";
  line2 += String(mlx.readAmbientTempC(), 1);
  line2 += "[C]";
  Serial.println(line1);
  Serial.println(line2);
  delay(2000);
}
