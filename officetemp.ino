#include <Adafruit_WINC1500.h>
#include <SPI.h>

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"// your network key Index number (needed only for WEP)
#include "Arduino.h"
#include "dht11.h"
#include "RTC.h"
#include "TempControl.h"
#include "avr/wdt.h"
#include "avr/eeprom.h"
#include "Adafruit_MCP9808.h"

#define VERBOSE

// Define the WINC1500 board connections below.
// If you're following the Adafruit WINC1500 board
// guide you don't need to modify these:
#define WINC_CS   8
#define WINC_IRQ  7
#define WINC_RST  4
#define WINC_EN   5     // or, tie EN to VCC and comment this out
// The SPI pins of the WINC1500 (SCK, MOSI, MISO) should be
// connected to the hardware SPI port of the Arduino.
// On an Uno or compatible these are SCK = #13, MISO = #12, MOSI = #11.
//   MEGA is 52 50 51 <--------------
// On an Arduino Zero use the 6-pin ICSP header, see:
//   https://www.arduino.cc/en/Reference/SPI

// Setup the WINC1500 connection with the pins above and the default hardware SPI.
Adafruit_WINC1500 WiFi(WINC_CS, WINC_IRQ, WINC_RST);

// Or just use hardware SPI (SCK/MOSI/MISO) and defaults, SS -> #10, INT -> #7, RST -> #5, EN -> 3-5V
//Adafruit_WINC1500 WiFi;


#include "/kghome.h"
int keyIndex = 0;

#define LOG_DATA Serial.print
#define LOG_DATA_LINE Serial.println

// Store the MQTT server, client ID, username, and password in flash memory.
const char MQTT_SERVER[]     = AIO_SERVER;

// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[]   = AIO_KEY;
const char MQTT_USERNAME[]   = AIO_USERNAME;
const char MQTT_PASSWORD[]   = AIO_KEY;

int status = WL_IDLE_STATUS;


// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
Adafruit_WINC1500Client client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/
const char TEMPERATURE_FEED[]  = AIO_USERNAME "/feeds/temperature";
Adafruit_MQTT_Publish temp_feed = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

const char HUMIDITY_FEED[]  = AIO_USERNAME "/feeds/humidity";
Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);

const char SETTEMP_FEED[]  = AIO_USERNAME "/feeds/office_temp";
Adafruit_MQTT_Subscribe settemp = Adafruit_MQTT_Subscribe(&mqtt, SETTEMP_FEED);

const char HVAC_FEED[]  = AIO_USERNAME "/feeds/hvac";
Adafruit_MQTT_Publish hvac_feed = Adafruit_MQTT_Publish(&mqtt, HVAC_FEED);

const char TIME_FEED[]  = AIO_USERNAME "/time/ISO-8601";
//Adafruit_MQTT_Subscribe timefeed = Adafruit_MQTT_Subscribe(&mqtt, TIME_FEED);

#define LED_PIN 13
RTC rtc;
TempControl control;
//BitBangSerial *bbs;

#ifdef LCD
SerialLCD *lcd;
#endif

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

dht11 DHT11;

#define DHT11PIN 2

#pragma pack(1)
struct day_sched {
  byte on_am;
  byte off_pm;
  byte temp;
};
struct EEPROM_DATA {
  uint32_t version;
  struct day_sched week[7];
};
struct EEPROM_DATA eedata = {
  0,
  {
    {0x50, 0xa0, 26}, //sunday
    {0x50, 0xa0, 26}, //mon
    {0x50, 0xa0, 26}, //tue
    {0x50, 0xa0, 26}, //wed
    {0x50, 0xa0, 26}, //thur
    {0x50, 0xa0, 26}, //fri
    {0x50, 0xa0, 26}, //sat
  }
};
#pragma pack(pop)

float CtoF(float celsius)
{
	float fahrenheit = (1.8 * celsius) + 32;
	return fahrenheit;
}

float thermostat(float *humidity)
{
  //LOG_DATA_LINE("\n");

  int chk = DHT11.read(DHT11PIN);

  //LOG_DATA("Read sensor: ");
  switch (chk)
  {
    case DHTLIB_OK:
      break;
    case DHTLIB_ERROR_CHECKSUM:
#ifdef VERBOSE
      LOG_DATA_LINE("Checksum error reading temp");
#endif
      return 0;
    case DHTLIB_ERROR_TIMEOUT:
#ifdef VERBOSE
      LOG_DATA_LINE("Time out error reading temp");
#endif
      return 0;
    default:
#ifdef VERBOSE
      LOG_DATA_LINE("Unknown error reading temp");
#endif
      return 0;
  }

  *humidity = DHT11.humidity;
  return DHT11.temperature;

}

int saveworktime;

unsigned long last = 0xffff0000;
unsigned long last_led = 0;
int led = 0;
#define LONG 1000
#define SHORT 400
unsigned long led_time = SHORT;
unsigned long sample_rate = 60000;
float curTemp = 26;

int last_ee = 0;
// check daily
unsigned long ee_rate = (unsigned)1000 * 60;
bool notemp;

/************************************************************************************************************************
   Setup Section
*/

void setup() {
  float request_temp;

  int resettype;

  resettype = MCUSR;
  MCUSR = 0;
  wdt_disable();

#ifdef WINC_EN
  pinMode(WINC_EN, OUTPUT);
  //  analogWrite(WINC_EN, 255);
  digitalWrite(WINC_EN, HIGH);
#endif

  //Initialize serial and wait for port to open:
  Serial.begin(9600);

#ifdef VERBOSE
  LOG_DATA_LINE("WINC1500 Web client test");
#endif
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
#ifdef VERBOSE
    LOG_DATA_LINE("WiFi shield not present");
#endif
    // don't continue:
    while (true);
  }
#ifdef VERBOSE
  LOG_DATA_LINE("WiFi shield found...");
#endif

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
#ifdef VERBOSE
    LOG_DATA("Attempting to connect to SSID: ");
    LOG_DATA_LINE(ssid);
#endif
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    uint8_t timeout = 10;
    while (timeout && (WiFi.status() != WL_CONNECTED)) {
      timeout--;
      delay(1000);
    }
  }

#ifdef LCD
  bbs = new BitBangSerial(4);
  lcd = new SerialLCD(2, 40, bbs);

  // Initialize LCD module
  lcd->init();

  lcd->setContrast(1);
  // Set Backlight
  lcd->setBacklightBrightness(1);
  lcd->clear();
  lcd->on();
#endif

  //IR Demo send a cmd To Mitsubishi HVAC
#ifdef VERBOSE
  LOG_DATA_LINE(F(""));
  LOG_DATA_LINE(F("Office temp solution"));
  LOG_DATA_LINE(F(""));
  LOG_DATA_LINE(F("DHT11 for humidity "));
  LOG_DATA_LINE(DHT11LIB_VERSION);
  LOG_DATA_LINE();
#endif

#ifdef VERBOSE
  LOG_DATA_LINE(F("Init RTC"));
#endif
  // initialize digital pin LED_PIN as an output.
  pinMode(LED_PIN, OUTPUT);
  rtc.setup();

  //  rtc.setDS3231time(0,30,7,RTC::DAY_WEDNESDAY,14,9,16);

#ifdef VERBOSE
  LOG_DATA_LINE("Temp setup...");
#endif
  if (!tempsensor.begin()) {
#ifdef VERBOSE
    LOG_DATA_LINE("Couldn't find MCP9808!");
#endif
    notemp = true;
  } else
    notemp = false;

  if (!notemp) {
    tempsensor.shutdown_wake(0);   // Don't remove this line! required before reading temp

#ifdef VERBOSE
    LOG_DATA("MCP9808 Current temp: ");

    LOG_DATA_LINE(tempsensor.readTempC(), 2);
    LOG_DATA_LINE("");
#endif
  }

  control.setTemp(26);
  wdt_enable(WDTO_4S);
  mqtt.subscribe(&settemp);

}

/************************************************************************************************************************
   WIFI Section
*/

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    LOG_DATA("Attempting to connect to SSID: ");
    LOG_DATA_LINE(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    uint8_t timeout = 10;
    while (timeout && (WiFi.status() != WL_CONNECTED)) {
      timeout--;
      delay(1000);
    }
  }

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  LOG_DATA("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    LOG_DATA_LINE(mqtt.connectErrorString(ret));
    LOG_DATA_LINE("Retrying MQTT connection in 2 seconds...");
    mqtt.disconnect();
    delay(2000);  // wait 5 seconds
  }
  LOG_DATA_LINE("MQTT Connected!");
}

/************************************************************************************************************************
   LOOP  Section
*/
void loop() {
  float curtemp;
  float humidity;
  unsigned long ms;
  int ok;
  MQTT_connect();

  // Get subscription
  Adafruit_MQTT_Subscribe *subscription;
  LOG_DATA("Wait for subscription");
  // this is our 'wait for incoming subscription packets' busy subloop
  while (subscription = mqtt.readSubscription(5000)) {

    // we only care about the lamp events
    if (subscription == &settemp) {

      // convert mqtt ascii payload to int
      char *value = (char *)settemp.lastread;
      LOG_DATA(F("Received: "));
      LOG_DATA_LINE(value);
      double temp = atof(value);

      LOG_DATA_LINE((float)temp);
      control.setTemp((float)temp);
    }
  }

  ms = millis();
//      LOG_DATA(ms);
//    LOG_DATA(" ");


#ifdef SERIAL_INPUT
  if (Serial.available() > 0) {
    int byte;
    byte = Serial.read();

    switch (byte) {
      case '=':
        control.reset();
        break;
      case '+':
        curTemp += .5;
        control.setTemp(curTemp);
        break;
      case '-':
        curTemp -= .5;
        control.setTemp(curTemp);
        break;

      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
#ifdef LCD
        lcd->setBacklightBrightness(byte - '0');
#endif
        break;
    }
  }
#endif
    static enum LED_PULSE { LED_START, LED_BLINK, LED_WAIT } led_state = LED_START;

  if (ms - last_led >= led_time) {
    static int pulse_count;

    last_led += led_time;
    switch(led_state) {
    case LED_START:
      led = 0;
      pulse_count = control.getstate()*2;
      led_time = LONG;
      led_state = LED_BLINK;
      LOG_DATA("LED_START:");
      LOG_DATA_LINE(pulse_count);
      break;
    case LED_BLINK:
      --pulse_count;
      led ^= 1;
      led_time = SHORT;
      if (pulse_count <= 0) led_state = LED_WAIT;
      LOG_DATA("LED_BLINK:");
      LOG_DATA(pulse_count);
      break;
    case LED_WAIT:
      led = 0;
      led_time = LONG;
      led_state = LED_START;
      LOG_DATA("LED_WAIT:");
      LOG_DATA(pulse_count);
      break;
    }
    digitalWrite(LED_PIN, led);
    wdt_reset();
#ifdef LCD
    lcd->clear();
    lcd->home();
    lcd->print(ms);
#endif
  }
#ifdef USE_EEPROM
  if (ms - last_ee >= ee_rate) {
    last_ee += ee_rate;
    if (eeprom_read_word(0) != eedata.version) {
      unsigned x;
      byte *buf = (byte *)&eedata;
      for (x = 0; x < sizeof(eedata); ++x) {
        if (eeprom_read_byte((uint8_t*)x) != *buf)
          eeprom_write_byte((uint8_t*)x, *buf);
        ++buf;
      }
    }
  }
#endif
  if (ms - last >= sample_rate && led_state == LED_WAIT) {
    last += sample_rate;
      LOG_DATA_LINE("poll");

    digitalWrite(LED_PIN, HIGH);
  LOG_DATA("... get rtc");
#ifdef VERBOSE
    rtc.displayTime(&Serial);
#endif

#ifdef VERBOSE
    LOG_DATA(" ");
    LOG_DATA(ms);
    LOG_DATA(" ");
#endif

#ifdef LCD
    lcd->home();
    rtc.displayTime(lcd);
#endif

    curtemp = thermostat(&humidity);
    if (curtemp == 0) {
      LOG_DATA_LINE("Temp failed");
      return;
    }
#ifdef VERBOSE
    LOG_DATA(" ");
    LOG_DATA((float)curtemp, 2);
    LOG_DATA("C(dht) ");
#endif

    if (!notemp) {
      tempsensor.shutdown_wake(0);
      // Read and print out the temperature, then convert to *F
      curtemp = tempsensor.readTempC();
      tempsensor.shutdown_wake(1);
    }

#ifdef SERIAL_OUT
    LOG_DATA((float)DHT11.humidity, 2);
    LOG_DATA("% ");
    LOG_DATA((float)curtemp, 2);
    LOG_DATA("C Req:");
#endif

    // Publish data
    float state = control.getstate();
    hvac_feed.publish(state);
    ok = temp_feed.publish(curtemp);
#ifdef VERBOSE
    if (!ok)
      LOG_DATA_LINE(F("Failed to publish temperature"));
    else
      LOG_DATA_LINE(F("Temperature published!"));
#endif
    humidity = (float)DHT11.humidity;
    ok = humidity_feed.publish(humidity);
#ifdef VERBOSE
    if (!ok)
      LOG_DATA_LINE(F("Failed to publish humidity"));
    else
      LOG_DATA_LINE(F("Humidity published!"));
#endif
    int hour = rtc.getHour();

    // 5am to 10pm
    enum RTC::DAYOFWEEK weekday = rtc.getDow();
    int worktime = (hour > 4 && hour < 20) && (weekday > RTC::DAYOFWEEK::DAY_MONDAY && weekday < RTC::DAY_FRIDAY);
    if (worktime != saveworktime)
      control.setWorktime(worktime);
    saveworktime = worktime;

    int set_temp = control.updateTemp(curtemp, humidity);
#ifdef SERIAL_OUTPUT
    LOG_DATA(" ");
    LOG_DATA(set_temp);

    LOG_DATA_LINE("");
#endif
    digitalWrite(LED_PIN, LOW);

  }

  /*  if (client.connected()) {
      // if there are incoming bytes available
      // from the server, read them and print them:
      while (client.available()) {
        char c = client.read();
        Serial.write(c);
      }
    }
    if (!client.connected()) {
    // if the server's disconnected, stop the client:
      LOG_DATA_LINE();
      LOG_DATA_LINE("disconnecting from server.");
      client.stop();
    }
  */
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  //  LOG_DATA("SSID: ");
  //  LOG_DATA_LINE(WiFi.SSID());

  // print your WiFi shield's IP address:
  //IPAddress ip = WiFi.localIP();
  //LOG_DATA("IP Address: ");
  //LOG_DATA_LINE(ip);

  // print the received signal strength:
  //  long rssi = WiFi.RSSI();
  //  LOG_DATA("signal strength (RSSI):");
  //  LOG_DATA(rssi);
  //  LOG_DATA_LINE(" dBm");

}
