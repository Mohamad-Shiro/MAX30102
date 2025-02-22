#include <DFRobot_MAX30102.h>
#include <ThingSpeak.h>
#include <WiFi.h>
#define WIFI_NETWORK "Shiro" 
#define WIFI_PASSWORD "Shiro@500" 
#define WIFI_TIMEOUT_MS 5000 
#define CHANNEL_ID 2851576 
#define CHANNEL_API "YJYX968AT0XUKIY2" 


// Task declarations 
void keepWiFiAlive(void * parameter); 
void sensorTask(void * parameter); 
void thigspeaktask(void * parameter); 
 
WiFiClient client;
DFRobot_MAX30102 particleSensor;

TaskHandle_t SensorTaskHandle; 

int spo2Value = -1;


void setup()
{
  //Init serial
  Serial.begin(115200);
  ThingSpeak.begin(client);
  while (!particleSensor.begin()) {
    Serial.println("MAX30102 was not found");
    delay(1000);
  }
  particleSensor.sensorConfiguration(/*ledBrightness=*/50, /*sampleAverage=*/SAMPLEAVG_4, \
                        /*ledMode=*/MODE_MULTILED, /*sampleRate=*/SAMPLERATE_100, \
                        /*pulseWidth=*/PULSEWIDTH_411, /*adcRange=*/ADCRANGE_16384);

  // Create the sensor task
  BaseType_t result = xTaskCreatePinnedToCore(
    sensorTask,           // Task function
    "Sensor Task",        // Name of the task
    4096,                 // Stack size in words
    NULL,                 // Parameter
    1,                    // Priority
    &SensorTaskHandle,     // Task handle
    0
  );
 
  xTaskCreatePinnedToCore( 
    keepWiFiAlive, 
    "KeepWiFiAlive", 
    10000, 
    NULL, 
    1, 
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  ); 

  xTaskCreatePinnedToCore( 
    thigspeaktask, 
    "thigspeaktask", 
    10000, 
    NULL, 
    1, 
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  ); 
}

int32_t SPO2; //SPO2
int8_t SPO2Valid; //Flag to display if SPO2 calculation is valid
int32_t heartRate; //Heart-rate
int8_t heartRateValid; //Flag to display if heart-rate calculation is valid 

void loop()
{
   // The main loop can be used for other tasks or remain idle
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to prevent loop hogging the CPU
}

void sensorTask(void *) {
  for (;;) {
    Serial.println(F("Wait about four seconds"));
    particleSensor.heartrateAndOxygenSaturation(/**SPO2=*/&SPO2, /**SPO2Valid=*/&SPO2Valid, /**heartRate=*/&heartRate, /**heartRateValid=*/&heartRateValid);
    //Print result 
    Serial.print(F("SPO2="));
    Serial.print(SPO2, DEC);
    Serial.print(F(", SPO2Valid="));
    Serial.println(SPO2Valid, DEC);

    if (SPO2Valid != 0) {
      spo2Value = SPO2;
    }
    // vTaskDelay(4000 / portTICK_PERIOD_MS);
  }
}

void keepWiFiAlive(void * parameter) { 
  for (;;) {  // Infinite loop to keep the task running 
    if (WiFi.status() == WL_CONNECTED) { 
      Serial.println("WiFi still connected."); 
      vTaskDelay(10000 / portTICK_PERIOD_MS); 
      continue; 
    } 
 
    Serial.println("WiFi Connecting..."); 
    WiFi.mode(WIFI_STA); 
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD); 
 
    unsigned long startAttemptTime = millis(); 
 
    // Attempt to connect within the timeout 
    while (WiFi.status() != WL_CONNECTED && 
           millis() - startAttemptTime < WIFI_TIMEOUT_MS) {} 
 
    if (WiFi.status() != WL_CONNECTED) { 
      Serial.println("[WIFI] FAILED"); 
      vTaskDelay(WIFI_TIMEOUT_MS / portTICK_PERIOD_MS); 
    } else { 
      Serial.println("[WIFI] Connected: " + WiFi.localIP().toString()); 
    } 
  } 
}

void thigspeaktask(void * parameter) { 
  for (;;) {  
 
    // Delay before reading (2 seconds) 
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (spo2Value != -1)
      ThingSpeak.writeField(CHANNEL_ID,1,spo2Value,CHANNEL_API);
      Serial.println("Message sent.");
    vTaskDelay(13000 / portTICK_PERIOD_MS);
  }
}
