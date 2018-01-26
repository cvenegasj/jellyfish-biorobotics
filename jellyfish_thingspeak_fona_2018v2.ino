/************************************************************************
 * For Arduino UNO
 * 
 * Pins for servo: 9
 * Pins for DS18B20 sensor: 5
 * Pins for Adafruit FONA 808: 2, 3, 4
 * 
 * Electronic components:
 * https://www.adafruit.com/product/2542
 * https://www.adafruit.com/product/381
 * 
 * 
 * 
 *************************************************************************/


#include <Servo.h>
// For Fona
#include "Adafruit_FONA.h"
// For temperature sensor
#include <DallasTemperature.h>

// Fona pins
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
// DS18B20 pin
#define ONE_WIRE_BUS 5


/***********************************************************************/
// Fona settings
// Default to using software serial. Hardware serial is also possible!
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

/***********************************************************************/
// DS18B20 sensor settings
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/***********************************************************************/


class Sweeper {
  Servo servo;              // the servo
  int pos;                  // current servo position 
  int increment;            // increment to move for each interval
  int  updateInterval;      // interval between updates
  unsigned long lastUpdate; // last update of position
  int sweepDirection;

  public: 
  Sweeper(int p_interval, int p_increment) {
    updateInterval = p_interval;
    increment = p_increment;
    pos = 80; // central position
  }
  
  void Attach(int pin) {
    servo.attach(pin);
  }
  
  void Detach() {
    servo.detach();
  }

  void SweepWithDirection(int sweepDir) { // 0 full | -1 left | 1 right
    switch (sweepDir) {
      case 0:
        MakeFullSweep();
        break;

      case -1:
        MakeLeftSweep();
        break;

      case 1:
        MakeRightSweep();
        break;
    }
  }

  void Update() {
    
  }

  void MakeFullSweep() {
    if (!servo.attached()) {
      servo.attach(9);
    }
    
    if ((millis() - lastUpdate) > updateInterval) {
      lastUpdate = millis();
      pos += increment;
      servo.write(pos);
      // end of sweep
      if ((pos >= 130) || (pos <= 30)) {
        // reverse direction
        increment = -increment;
      }
    }
  }

  void MakeLeftSweep() {
    if (!servo.attached()) {
      servo.attach(9);
    }

    if (pos < 80) {
      pos = 80;
    }

    if ((millis() - lastUpdate) > updateInterval) {
      lastUpdate = millis();
      pos += increment;
      servo.write(pos);
      // end of sweep
      if ((pos >= 130) || (pos <= 80)) {
        // reverse direction
        increment = -increment;
      }
    }
  }

  void MakeRightSweep() {
    if (!servo.attached()) {
      servo.attach(9);
    }

    if (pos > 80) {
      pos = 80;
    }

    if ((millis() - lastUpdate) > updateInterval) {
      lastUpdate = millis();
      pos -= increment;
      servo.write(pos);
      // end of sweep
      if ((pos >= 80) || (pos <= 30)) {
        // reverse direction
        increment = -increment;
      }
    }
  }

  void TerminateSweep() {
    if (!servo.attached()) {
      return;
    }
    /*
    delay(15);
    pos = 80;
    servo.write(pos);
    delay(15);
    servo.detach(); */
    
    unsigned long currentMillis = millis();
    // return to initial central position
    if (currentMillis - lastUpdate > 60) {
      servo.detach();
    } else if ((currentMillis - lastUpdate) > 30) { // 30ms is ok to update the servo position
      pos = 80;
      servo.write(pos);
    }
  }

  int GetUpdateInterval() {
    return updateInterval;
  }

  void SetUpdateInterval(int newUpdateInterval) {
    updateInterval = newUpdateInterval;
  }

};


/***********************************************************************/

class TemperatureManager {

  public:
  TemperatureManager() {
    
  }

  void Init() {
    sensors.begin();
  }

  float GetData() {
    sensors.requestTemperatures(); 
    return sensors.getTempCByIndex(0);
  }
  
};


/***********************************************************************/

class FonaManager {
  float latitude;
  float longitude;
  
  public:
  FonaManager() {
    
  }

  float GetLatitude() {
    return latitude;
  }

  float GetLongitude() {
    return longitude;
  }

  void Init() {
    Serial.println(F("Initializing FONA....(May take 3 seconds)"));
    fonaSerial->begin(4800);
    if (!fona.begin(*fonaSerial)) {
      Serial.println(F("Couldn't find FONA"));
      while(1);
    }
    Serial.println(F("FONA is OK"));

    //delay(2000);
    // Try to enable GPRS
    //if (!fona.enableGPRS(true))
    //  Serial.println(F("GPRS Failed to turn on"));
    
    if (!fona.enableGPS(true))
      Serial.println(F("GPS Failed to turn on"));
  }

  void EnableGPRS() {
    if (!fona.enableGPRS(true))
      Serial.println(F("GPRS Failed to turn on"));
  }

  void EnableGPS() {
    if (!fona.enableGPS(true))
      Serial.println(F("GPS Failed to turn on"));
  }

  void RequestGSMLocation() {
    // Check for network, then GPRS 
    if (fona.getNetworkStatus() == 1) {
      boolean gsmloc_success = fona.getGSMLoc(&latitude, &longitude);
      if (gsmloc_success) {
        Serial.print("GSMLoc lat:");
        Serial.println(latitude, 6);
        Serial.print("GSMLoc long:");
        Serial.println(longitude, 6);
      } else {
        Serial.println("GSM location failed...");
      }
    }
  }

  void SendDataPost(float lat, float lon, float temp) {
    // Post data to thingspeak
    uint16_t statuscode;
    int16_t length;
    char url[80] = "api.thingspeak.com/update";
    char data[250];
    // field1: temperature | field2: turbidity | field3: As | field4: lat | field5: lon
    String dataString = "key=5FUREQL1V57LO9TU&field1=" + String(temp, 1);
    dataString += "&field4=" + String(lat, 6); // 6 decimal places
    dataString += "&field5=" + String(lon, 6);
    dataString.toCharArray(data, 250);
    Serial.println(data);

    if (!fona.HTTP_POST_start(url, F("application/x-www-form-urlencoded"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
      Serial.println("Failed POST request!");
      return;
    }
    while (length > 0) {
      while (fona.available()) {
        char c = fona.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
        loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
        UDR0 = c;
#else
        Serial.write(c);
#endif
  
          length--;
          if (! length) break;
        }
      }
      Serial.println(F("\nEnded POST request****"));
      fona.HTTP_POST_end();
  }

  void SendDataGet() {
    uint16_t statuscode;
    int16_t length;
    char url[120];
    String dataString = "api.thingspeak.com/update?api_key=5FUREQL1V57LO9TU&field1=" + String(6);
    //data += "&field4=" + String(latitude, 6); // 6 decimal digits
    //data += "&field5=" + String(longitude, 6);
    dataString.toCharArray(url, 120);
    Serial.println(url);

    if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
      Serial.println("Failed GET request!");
      return;
    }
    while (length > 0) {
      while (fona.available()) {
        char c = fona.read();

        // Serial.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
        loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
        UDR0 = c;
#else
        Serial.write(c);
#endif
        length--;
        if (!length) break;
      }
    }
    Serial.println(F("\nEnded GET request****"));
    fona.HTTP_GET_end();
  }
  
};


/***********************************************************************/

// Sweeper manager
int sweeperUpdateInterval = 5;
int sweeperIncrement = 2;
Sweeper sweeper(sweeperUpdateInterval, sweeperIncrement); // passing updateInterval
// Fona manager
FonaManager fonaManager;
// Temperature manager
TemperatureManager tempManager;

unsigned long previousMillis1 = 0;

boolean isWaterDetected = false;
int sweepDirectionNumber = random(-1, 2);
uint32_t timer = millis();
  
void setup() {
  Serial.begin(115200);
  fonaManager.Init();
  tempManager.Init();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Water detection
  int water = analogRead(1);
  if (water < 600) {
    isWaterDetected = true;
    sweeper.SweepWithDirection(0);

    /*if ((currentMillis - previousMillis1) >= 3000) {
      previousMillis1 = currentMillis;
      sweepDirectionNumber = random(-1, 2);
    }*/
  } else {
    isWaterDetected = false;
    sweeper.TerminateSweep();
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // executes each n miliseconds
  if (millis() - timer > 10000) {
    sweeper.Detach(); 
    timer = millis(); // reset the timer
    fonaManager.EnableGPRS();
    fonaManager.RequestGSMLocation();
    Serial.print("Latitude: ");
    Serial.print(fonaManager.GetLatitude(), 6);
    Serial.print(", Longitude: ");
    Serial.println(fonaManager.GetLongitude(), 6);
    fonaManager.SendDataPost(fonaManager.GetLatitude(), fonaManager.GetLongitude(), tempManager.GetData());
  }
}
