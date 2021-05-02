#include <Adafruit_Sensor.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>

TwoWire I2CMLX = TwoWire(1);
Adafruit_MLX90614 mlx = Adafruit_MLX90614(0x5A, &I2CMLX);

TwoWire I2CMAX30105 = TwoWire(2);
MAX30105 particleSensor;

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

int accel_sensor_value_x = 0;
int accel_sensor_value_y = 0;
int accel_sensor_value_z = 0;

float ambientTemp = 0;
float objectTemp = 0;

bool deviceConnected = false;


#define I2C_SDA 21 //
#define I2C_SCL 22 //
#define I2C_MAX30105_SCL 18
#define I2C_MAX30105_SDA 19

#define NUMBER_OF_SENSORS 4

union multi_sensor_data
{
  struct __attribute__( ( packed ) )
  {
    uint8_t values[NUMBER_OF_SENSORS];
  };
  uint8_t bytes[ NUMBER_OF_SENSORS * sizeof( float ) ];
};

union multi_sensor_data multiSensorData;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "d5875408-fa51-4763-a75d-7d33cecebc31"
#define CHARACTERISTIC_UUID "a4f01d8c-a037-43b6-9050-1876a8c23584"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  //Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
  //To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
  uint16_t irBuffer[100]; //infrared LED sensor data
  uint16_t redBuffer[100];  //red LED sensor data
#else
  uint32_t irBuffer[100]; //infrared LED sensor data
  uint32_t redBuffer[100];  //red LED sensor data
#endif

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

BLECharacteristic *multiSensorDataCharacteristic;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

float accel, total, threshold, hysteresis;
int count, stepcount;
bool state, laststate;

void setup() {
  Serial.begin(115200);

  // Initialize heart ratesensor
  /*
  I2CMAX30105.begin(I2C_MAX30105_SDA, I2C_MAX30105_SCL, 0x57);
  if (!particleSensor.begin(I2CMAX30105, 50000, 0x57))
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
  }
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
  I2CMLX.begin(I2C_SDA, I2C_SCL, 0x5A);
  if (!mlx.begin()) {
    Serial.println("unable to start mlx");  
  }
  */

  // Create the BLE Device
  BLEDevice::init("Up Bluetooth Device");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  multiSensorDataCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  multiSensorDataCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  if (deviceConnected) {
    /*
    //bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
    bufferLength = 25;
    //read the first 100 samples, and determine the signal range
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) {//do we have new data?
        particleSensor.check(); //Check the sensor for new data
      }
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      if ((i - 1) % 50 == 0) {
        Serial.print(i - 1); Serial.println("%");
        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.println(irBuffer[i], DEC); 
      }
    }
  
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    //send samples and calculation result to terminal program through UART

    if (validHeartRate == 1) {
      multiSensorData.values[9] = int(heartRate);
      multiSensorData.values[10] = int(validHeartRate);
      Serial.print(F(", HR="));
      Serial.println(heartRate, DEC);
    } else {
      multiSensorData.values[9] = 0;
      multiSensorData.values[10] = int(validHeartRate);
    }

    if (validSPO2 == 1) {
      multiSensorData.values[11] = int(spo2);
      multiSensorData.values[12] = int(validSPO2);
      Serial.print(F(", SPO2="));
      Serial.println(spo2, DEC);
    } else {
      multiSensorData.values[11] = 0;
      multiSensorData.values[12] = int(validSPO2);       
    }

    delay(1000);
    
    ambientTemp = mlx.readAmbientTempC();
    objectTemp = mlx.readObjectTempC();
    Serial.println(ambientTemp);
    Serial.println(objectTemp);
    */
    accel_sensor_value_x = analogRead(34); // for universal, x is 32
    accel_sensor_value_y = analogRead(32); // for universal, y is 35
    accel_sensor_value_z = analogRead(35); //for universal, z is 34

    accel = sqrt( sq(accel_sensor_value_x) + sq(accel_sensor_value_y) + sq(accel_sensor_value_z) );

    // 閾値設定
    if (count < 100) {
      total += accel;
      count++;
    } else {
      Serial.println(total);
      Serial.println(count);
      threshold = total / count;
      hysteresis = threshold / 10;
      total = 0;
      count = 0;
      Serial.println(threshold + hysteresis);
    }
    //閾値判定
    if (accel > (threshold + hysteresis)) {
      state = true;
    } else if (accel < (threshold - hysteresis)) {
      state = false;
    }
    //歩数カウント
    if (laststate == false && state == true) {
      stepcount++;
      laststate = state;
    } else if (laststate == true && state == false) {
      laststate = state;
    }

    multiSensorData.values[0] = accel_sensor_value_x;
    multiSensorData.values[1] = accel_sensor_value_y;
    multiSensorData.values[2] = accel_sensor_value_z;
    // 今後のために一応残しておく
    //multiSensorData.values[3] = int(bme.temperature);
    //multiSensorData.values[4] = int(bme.humidity);
    //multiSensorData.values[5] = int(bme.gas_resistance / 1000.0);
    //multiSensorData.values[6] = int(bme.pressure / 1000.0); // hectopascalではなく1000で割っている
    //multiSensorData.values[7] = int(ambientTemp);
    //multiSensorData.values[8] = int(objectTemp);
    multiSensorData.values[9] = stepcount;
    multiSensorDataCharacteristic->setValue( multiSensorData.bytes, sizeof multiSensorData.bytes );
    /*
    Serial.println(sizeof multiSensorData.bytes);
    for ( int i = 0; i < sizeof multiSensorData.bytes; i++ )
    {
      Serial.print(multiSensorData.values[i]);
      Serial.print(" ");
    }
    Serial.println();
    */
    
    
    multiSensorDataCharacteristic->notify();
    
  }
  delay(10);
}
