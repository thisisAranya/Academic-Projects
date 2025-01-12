//Arduino Cloud
#include "arduino_secrets.h" 
#include "thingProperties.h"

// Dht11 sensor
#include <Adafruit_Sensor.h>
#include "DHT.h"

// Bmp280 sensor
#include <Adafruit_BMP280.h>

// DS18B20 sensor
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pulse Oxymeter
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

//lcd
#include <LiquidCrystal_I2C.h>

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;


// main code
#define REPORTING_PERIOD_MS 3000  // main
#define EMAIL_UPDATE_PERIOD_MS 3*60*1000 // main
#define WAIT_PERIOD 10000 //main

// Dht11 sensor
#define DHTPIN 17 // Replace with the actual pin to which the DHT11 sensor is connected
#define DHTTYPE DHT11

// Ds18B20 sensor
#define ONE_WIRE_BUS 15  // Pin for the DS18B20 sensor

// BMP280 sensor
#define SEALEVELPRESSURE_HPA (1017.9) // Change this to the current sea-level pressure in your location

// Flame sensor
#define Flame_PIN 14  // ESP32's pin GPIO14 connected to DO pin of the MQ2 sensor

// Buzzer
#define Buzzer_PIN 18

// MQ2 sensor
#define MQ2_PIN 27

// Max30102 sensor
#define MAX_BRIGHTNESS 255  // Pulse Oxymeter


// Creating Class Objects


// Pulse Oxymeter
MAX30105 particleSensor; 

// Ds18b20 sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Dht11 sensor
DHT dht(DHTPIN, DHTTYPE);

// BMP280 sensor
Adafruit_BMP280 bmp; 

//lcd 
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);


// Main Code
uint32_t tsLastReport = 0, tsLastEmail = 0;  // main

// Pulse Oxymeter start
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value calcualated as per Maxim's algorithm
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 2; //onboard led on esp32 nodemcu
byte readLED = 19; //Blinks with each data read 

long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute; //stores the BPM as per custom algorithm
int beatAvg = 0, sp02Avg = 0, heartRateAvg = 0, print_bpm = 0;//stores the average BPM and SPO2 
float ledBlinkFreq; //stores the frequency to blink the pulseLED
// Pulse Oxymeter End

 
void setup()
{ 
  Serial.begin(115200);   // main

  // Pulse Oxymeter start
  ledcSetup(0, 0, 8); // PWM Channel = 0, Initial PWM Frequency = 0Hz, Resolution = 8 bits
  ledcAttachPin(pulseLED, 0); //attach pulseLED pin to PWM Channel 0
  ledcWrite(0, 255); //set PWM Channel Duty Cycle to 255
  // Pulse Oxymeter end

  // Arduino Cloud
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  // Flame Ssensor
  pinMode(Flame_PIN, INPUT);

  // MQ2 Ssensor
  pinMode(MQ2_PIN, INPUT);

  // Buzzer
  pinMode(Buzzer_PIN, OUTPUT);

  // Ds18b20 sensor
  sensors.begin();

  // Dht11 sensor
  dht.begin();

  // BMP280 sensor
  if (!bmp.begin(0x76))
    {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        // while (1)
            ;
    }

  Serial.println("BMP280 sensor found.");
  
  // Pulse Oxymeter starts
  Serial.print("Initializing Pulse Oximeter..");
  
    // Initialize Pulse Oxymeter sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }


  detected = false; //main
  start_time = 0; //main

  /*The following parameters should be tuned to get the best readings for IR and RED LED. 
   *The perfect values varies depending on your power consumption required, accuracy, ambient light, sensor mounting, etc. 
   *Refer Maxim App Notes to understand how to change these values
   *I got the best readings with these values for my setup. Change after going through the app notes.
   */
  byte ledBrightness = 50; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  // Pulse Oxymeter ends

  // turn on LCD backlight 
  lcd.begin();                   
  lcd.backlight();
}
 
void loop()
{
  // Pulse Oxymeter
  
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
  
    redBuffer[i] = particleSensor.getIR();
    irBuffer[i] = particleSensor.getRed();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  
    Serial.print(F("red: "));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F("\t ir: "));
    Serial.println(irBuffer[i], DEC);
  }
  
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Pulse Oxymeter
  
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {

    // Pulse Oxymeter

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
    
      //digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
    
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      //Uncomment these statements to view the raw data during calibration of sensor.
      //When uncommented, beatsPerMinute will be slightly off.
      /*Serial.print(F("red: "));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F("\t ir: "));
      Serial.print(irBuffer[i], DEC);
      Serial.print(F("\t HR="));
      Serial.print(heartRate, DEC);
      Serial.print(F("\t"));
      Serial.print(beatAvg, DEC);
      
      Serial.print(F("\t HRvalid="));
      Serial.print(validHeartRate, DEC);
      
      Serial.print(F("\t SPO2="));
      Serial.print(spo2, DEC);
      
      Serial.print(F("\t SPO2Valid="));
      Serial.println(validSPO2, DEC);*/

      long irValue = irBuffer[i];

      //Calculate BPM independent of Maxim Algorithm. 
      if (checkForBeat(irValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();
      
        beatsPerMinute = 60 / (delta / 1000.0);
        beatAvg = (beatAvg+beatsPerMinute)/2;

        if(beatAvg != 0)
          ledBlinkFreq = (float)(60.0/beatAvg);
        else
          ledBlinkFreq = 0;
        ledcWriteTone(0, ledBlinkFreq);
      }
      if(millis() - lastBeat > 10000)
      {
        beatsPerMinute = 0;
        beatAvg = (beatAvg+beatsPerMinute)/2;
        
        if(beatAvg != 0)
          ledBlinkFreq = (float)(60.0/beatAvg);
        else
          ledBlinkFreq = 0;
        ledcWriteTone(0, ledBlinkFreq);
      }
    }
  
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    

    //Calculates average HeartRate to display smooth transitions on Web Dashboard
    if(validHeartRate == 1 && heartRate < 120 && heartRate > 0)
    {
      heartRateAvg = (4*heartRateAvg+heartRate)/5;
    }
    else
    {
      heartRate = 0;
      heartRateAvg = (4*heartRateAvg+heartRate)/5;
    }

    //Calculates average SPO2 to display smooth transitions on Blynk App
    if(validSPO2 == 1 && spo2 < 100 && spo2 > 0)
    {
      sp02Avg = (sp02Avg+spo2)/2;
    }
    else
    {
      spo2 = 0;
      sp02Avg = (sp02Avg+spo2)/2;;
    }

    // Pulse Oxymeter end


    // main function

    if (millis() - tsLastReport > REPORTING_PERIOD_MS)
    { 
      ArduinoCloud.update();
      DHT_sensor_READ();
      BMP280_sensor_READ();
      FLAME_sensor_READ();
      MQ2_sensor_READ();
      DS18B20_SENSOR_READ();
      MAX30102_sensor_READ(heartRateAvg, sp02Avg);
      critical_condition_alarm();
      lcd_show();
      //lcd_show1();
      
      // if (millis() - tsLastEmail > 22*1000) {
      //   send_email = HIGH;
      //   Serial.println("entered");
      //   tsLastEmail = millis();
      // } else {
      //   send_email = LOW;
      //   Serial.println("did not enter");
      // }
    
      tsLastReport = millis();
    }
  }
}


void lcd_show() {
  // Clear the LCD
  lcd.clear();

  // Set cursor to first column, first row
  lcd.setCursor(0, 0);
  
  // Print bpm
  lcd.print("BPM:");
  lcd.print(int(cloud_bpm));  // Convert to int

  // Print spo2
  lcd.print(" SPO2:");
  lcd.print(int(cloud_spo2));  // Convert to int
  lcd.print("%");
  
  // Set cursor to first column, first row
  lcd.setCursor(0, 1);

  lcd_blink ^= 1; //toggle

  if(lcd_blink) {
    
    // Print temperature
    lcd.print("T:");
    lcd.print(int(dht.readTemperature()));  // Convert to int
    lcd.print("C");

    // Print humidity
    lcd.print(" H:");
    lcd.print(int(dht.readHumidity()));  // Convert to int
    lcd.print("%");
  } else {
    
    // Print pressure
    //float cloud_body_temperature1;
    
    if(cloud_body_temperature < 87.8) {
      cloud_body_temperature1 = 0;
    } else if (cloud_body_temperature > 97) {
      cloud_body_temperature1 = 99;
    } else {
      cloud_body_temperature1 = cloud_body_temperature*1.025;
    }
      
    
    lcd.print("BT:");
    //lcd.print(int(cloud_body_temperature1));  // Convert to int
    lcd.print(cloud_body_temperature1, 1);
    lcd.print("F");

    // Print pressure
    lcd.print(" P:");
    lcd.print(int(bmp.readPressure() / 1330));  // Convert to int
    lcd.print("cm");

  }
}

// void lcd_show() {
//   // Clear the LCD
//   lcd.clear();

//   // Set cursor to first column, first row
//   lcd.setCursor(0, 0);
  
//   // Print bpm
//   lcd.print("BPM:");
//   lcd.print(int(cloud_bpm));  // Convert to int

//   // Print spo2
//   lcd.print(" SPO2:");
//   lcd.print(int(cloud_spo2));  // Convert to int
//   lcd.print("%");
  
//   // Set cursor to first column, first row
//   lcd.setCursor(0, 1);

//   // Print temperature
//   lcd.print("T:");
//   lcd.print(int(bmp.readTemperature()));  // Convert to int
//   lcd.print("C");

//   // Print humidity
//   lcd.print(" H:");
//   lcd.print(int(dht.readHumidity()));  // Convert to int
//   lcd.print("%");

//   delay(4000);
// }


// void lcd_show1() {
//   // Clear the LCD
//   lcd.clear();

//   // Set cursor to first column, first row
//   lcd.setCursor(0, 0);
  
//   // Print bpm
//   lcd.print("BPM:");
//   lcd.print(int(cloud_bpm));  // Convert to int

//   // Print spo2
//   lcd.print(" SPO2:");
//   lcd.print(int(cloud_spo2));  // Convert to int
//   lcd.print("%");
  
//   // Set cursor to first column, first row
//   lcd.setCursor(0, 1);

//   // Print pressure
//   lcd.print("P:");
//   lcd.print(int(bmp.readPressure() / 1330));  // Convert to int
//   lcd.print("cmHg");

//   // Print pressure
//   float cloud_body_temperature1;
  
//   if(cloud_body_temperature < 90)
//     cloud_body_temperature1 = 0;
//   else
//     cloud_body_temperature1 = cloud_body_temperature*1.025;
  
//   lcd.print("  BT:");
//   lcd.print(int(cloud_body_temperature1));  // Convert to int
//   lcd.print("F");
// }


void MAX30102_sensor_READ(int heartRateAvg, int sp02Avg) {
  int ir = particleSensor.getIR();

  if (ir > 50000) {
    cloud_finger = HIGH;

    int print_bpm = heartRateAvg * 0.8;

    cloud_bpm = print_bpm;
    
    Serial.print(F("\nBPM="));
    Serial.print(print_bpm, DEC);
    
    cloud_spo2 = sp02Avg;
    
    Serial.print(F("\tSPO2="));
    Serial.print(sp02Avg, DEC);
  } 
  else {
    cloud_finger = LOW;
    cloud_bpm = 0;
    cloud_spo2 = 0;
  }
}

void FLAME_sensor_READ() {
  int flameState = digitalRead(Flame_PIN);

  if (flameState == HIGH) {
    cloud_flame = LOW;
    Serial.println("No flame detected");
  }
  else {
    cloud_flame = HIGH;
    Serial.println("Flame detected");
  }
}

void MQ2_sensor_READ() {
  int gasState = digitalRead(MQ2_PIN);

  if (gasState == HIGH) {
    cloud_gas = LOW;
    Serial.println("No toxic detected");
  }
  else {
    cloud_gas = HIGH;
    Serial.println("Toxic Gas detected");
  }
}

void BMP280_sensor_READ() {
  float p = bmp.readPressure()/133.0;
  float ht = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  cloud_pressure = p;

  Serial.print("Pressure: ");Serial.print(p);Serial.println("mmHg");
  Serial.print("Altitude: ");Serial.print(ht);Serial.println("m");
}

void DHT_sensor_READ() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  cloud_humidity = h;
  cloud_temperature = t;

  Serial.print("Temperature: ");Serial.print(t);Serial.println("°C");
  Serial.print("Humidity: ");Serial.print(h);Serial.println("%");
}

void DS18B20_SENSOR_READ() {
  sensors.requestTemperatures();  // Request temperature readings
  
  float temperatureC = sensors.getTempCByIndex(0);  // Get temperature in degrees Celsius
  float temperatureF = 1.8*temperatureC + 32;

  cloud_body_temperature = temperatureF;

  if (temperatureC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");Serial.print(temperatureF);Serial.println(" °F");
  } else {
    Serial.println("Error reading temperature!");
  }
}

// void onCloudAlarmChange() {
//   // Add your code here to act upon CloudAlarm change
//   bool alarm = cloud_alarm;

//   // Update the buzzer based on the new state
//   if (alarm) {
//     digitalWrite(Buzzer_PIN, HIGH);  // Turn on the buzzer
//   } else {
//     digitalWrite(Buzzer_PIN, LOW);   // Turn off the buzzer
//   }
// }

void critical_condition_alarm() {
  bool bpm_alert;
  int bpm_max_limit = 100;
  int bpm_min_limit = 40;

//  if (cloud_finger == true && detected == false) {
//    detected = true;
//    start_time = millis();
//  } else {
//    detected = false;
//  }

  if ((cloud_bpm > bpm_max_limit || (cloud_bpm < bpm_min_limit)) && cloud_finger == true) {   
    bpm_alert = true;
  } else {
    bpm_alert = false;
  }

  bool spo2_alert;
  int spo2_min_limit = 80;
  if (cloud_spo2 < spo2_min_limit && cloud_finger == true) {
    spo2_alert = true;
  } else {
    spo2_alert = false;
  }

  bool temperature_alert;
  int temperature_min_limit = 20;
  int temperature_max_limit = 36;
  if (cloud_temperature > temperature_max_limit || cloud_temperature < temperature_min_limit ) {
    temperature_alert = true;
  } else {
    temperature_alert = false;
  }

  bool humidity_alert;
  int humidity_min_limit = 30;
  int humidity_max_limit = 85;
  if (cloud_humidity > humidity_max_limit || cloud_humidity < humidity_min_limit ) {
    humidity_alert = true;
  } else {
    humidity_alert = false;
  }

  bool pressure_alert;
  int pressure_min_limit = 745;
  int pressure_max_limit = 775;
  if (cloud_pressure > pressure_max_limit || cloud_pressure < pressure_min_limit ) {
    pressure_alert = true;
  } else {
    pressure_alert = false;
  }

  if (bpm_alert || spo2_alert || temperature_alert || humidity_alert || pressure_alert || cloud_flame || cloud_gas) {
    cloud_alarm = true;
    digitalWrite(Buzzer_PIN, HIGH);
  } else {
    cloud_alarm = false;
    digitalWrite(Buzzer_PIN, LOW);
  }

}
