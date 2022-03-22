#include <EEPROM.h>
#include <AccelStepper.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include "RTClib.h"
#include <WiFiUdp.h>

// mgtt
#include <PubSubClient.h>

#include "secrets.h"

// https://diyi0t.com/segment-led-display-tutorial-for-arduino-and-esp8266/
// https://i2.wp.com/randomnerdtutorials.com/wp-content/uploads/2019/05/ESP8266-WeMos-D1-Mini-pinout-gpio-pin.png?w=715&quality=100&strip=all&ssl=1
// https://codebender.cc/sketch:431066#home%20pin%20example%20using%20AccelStepper.ino


WiFiClient wifi;  // or WiFiClientSecure for HTTPS
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* device_name = "MainHeater";

short eeprom_magic_number = 20437;


// curl 'http://api.openweathermap.org/data/2.5/forecast?lat=OPENWEATHERMAP_LAT&lon=OPENWEATHERMAP_LON&units=metric&appid=$OPENWEATHERMAP_TOKEN' | python -m json.tool
const char* openweathermap_url = "http://api.openweathermap.org/data/2.5/forecast?lat=" OPENWEATHERMAP_LAT "&units=metric&lon=" OPENWEATHERMAP_LON "&appid=" OPENWEATHERMAP_TOKEN;
HTTPClient http;

// https://roelofjanelsinga.com/articles/mqtt-discovery-with-an-arduino/
// https://pubsubclient.knolleary.net/
const char* mqtt_server = "192.168.0.10";
const int mqtt_port = 1883;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;
const char* state_topic = "home/heaters/main/state";
const char* set_override_topic = "home/heaters/main/set_override";
const char* set_override_cycles_topic = "home/heaters/main/set_override_cycles";
const char* set_led_topic = "home/heaters/main/set_led";

const char* mqtt_name = "Main Heater";

PubSubClient mqtt(wifi);

struct Forecast {
  float temp;
  float sun;
};

// hourly caching
Forecast cached_forecast = { -1, -1};
int cached_forecast_hour = -1;
// get min temp between these hours
int forecast_start_hour = 8;
int forecast_stop_hour = 18;

WiFiUDP Udp;
WiFiUDP ntpUdp;
IPAddress ip(192, 168, 0, 201);
IPAddress dns(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

unsigned int localUdpPort = 4210;  // local port to listen on
// https://www.ntppool.org/zone/europe
NTPClient ntp(ntpUdp, "ie.pool.ntp.org");

const int stop_hour = 8; // 24h hour when heater should be turned off
const int rows = 3;
const int columns = 2;
int temp_hours[ rows ][ columns ] = {
  // { temp, hours }
  { 15, 3 },
  { 10, 4 },
  { 5, 6 },
};
const int moderate_hours = 4;  // how many hours is considered a moderate use (>moderate: PURPLE else BLUE)
const float sun_temp_coeficient = 2;  // temp increase coeficient for every 4 hours of sun (adjust effective temperature based on heat from sun)

short heater_state = -1;  // -1: undefined, 0: off, 1: on
short override = -1;  // 1-: disabled, 0: heater off, else: number of target hours
long long override_end = 0; // 0: never stops, else: unix timestamp of last stop cycle
short led = 1;  // led light on/off

// ULN2003 Motor Driver Pins
#define IN1 5
#define IN2 4
#define IN3 14
#define IN4 12
#define HOMING_PIN 13
// initialize the stepper library
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

#define RED 0
#define GREEN 15
#define BLUE 16


void heater_off() {
  Serial.println("Turning heater OFF");
  long initial_homing = -1;
  // set the speed and acceleration
  //  Set Max Speed and Acceleration of each Steppers at startup for homing
  stepper.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100.0);  // Set Acceleration of Stepper

  // set target position
  Serial.print("Stepper is Homing . . . . . . . . . . . ");
  while (digitalRead(HOMING_PIN) == HIGH) {
    stepper.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    stepper.run();  // Start moving the stepper
    do_concurrent_tasks();
    delay(5);
  }
  Serial.print("DONE\r\nStepper findigin homming limit . . . . . . . . . . . ");
  stepper.setCurrentPosition(0);  // Set the current position as zero for now
  initial_homing = 1;
  while (digitalRead(HOMING_PIN) == LOW) { // Make the Stepper move CW until the switch is deactivated
    stepper.moveTo(initial_homing);
    stepper.run();
    initial_homing++;
    do_concurrent_tasks();
    delay(5);
  }
  Serial.print("DONE\r\nStepper moving to homing offset . . . . . . . . . . . ");
  // offset some steps to make sure heater is in a closed position
  stepper.setCurrentPosition(0);
  stepper.moveTo(-120);
  while (stepper.distanceToGo()) {
    stepper.run();
    do_concurrent_tasks();
  }
  stepper.setCurrentPosition(0);
  heater_state = 0;
  Serial.println("Homing Completed");
}

void heater_on() {
  Serial.println("Turning heater ON");
  if (heater_state == -1) {
    Serial.println(" . Stepper needs homming:");
    heater_off();
  } else if (heater_state == 1) {
    Serial.println("WARNING: heater already ON");
  }
  stepper.moveTo(3 * 1024 - 100);
  while (stepper.distanceToGo()) {
    stepper.run();
    do_concurrent_tasks();
  }
  heater_state = 1;
  Serial.println("Heater ON completed");
}


Forecast get_forecast(DateTime now) {
  if (cached_forecast_hour != now.hour() || cached_forecast.temp == -1) {
    Serial.println("forecast cache expired, fetching ...");
    // keep clock in sync
    ntp.update();
    Forecast forecast = fetch_forecast(now);
    if (forecast.temp == -1) {
      // error fetching temp
      return { -1, -1};
    }
    cached_forecast = forecast;
    cached_forecast_hour = now.hour();
    publish_mqtt_state(forecast);
  }
  return cached_forecast;
}


Forecast fetch_forecast(DateTime now) {
  float temp_min = -1;
  float sun = -1;
  int sunset = 0;

  if (WiFi.status() != WL_CONNECTED) {
    if (reconnect_wifi() == -1) {
      return { -1, -1};
    }
  }

  // cast to long instead of time_t because - operations with unsigned lead to unexpected values
  long long next_stop;
  get_next_stop_time(now, &next_stop);
  long long start_time = next_stop + ((forecast_start_hour - stop_hour) * 60 * 60);
  long long stop_time = next_stop + ((forecast_stop_hour - stop_hour) * 60 * 60);
  Serial.print("start_time: ");
  Serial.println(start_time);
  Serial.print("stop_time: ");
  Serial.println(stop_time);

  if (((DateTime) next_stop).day() != now.day()) {
    Serial.println("checking forecast for tomorrow");
    sunset += 24 * 60 * 60;
  }

  Serial.println("Connecting to api.openweathermap.org ....");
  int err = 0;
  http.useHTTP10(true);
  if (http.begin(wifi, openweathermap_url)) {
    Serial.print("[HTTP] GET...\n");
    int httpCode = http.GET();
    if (httpCode > 0) {
      Serial.printf("[HTTP] GET... code: %d\n", httpCode);
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        DynamicJsonDocument doc(24576);
        DeserializationError error = deserializeJson(doc, http.getStream());
        if (error) {
          Serial.print("Deserialization failed with code: ");
          Serial.println(error.c_str());
        } else {
          JsonArray list = doc["list"].as<JsonArray>();
          float temp;
          long long dt;
          int clouds;
          sunset += (long long)doc["city"]["sunset"];
          sun = 0.0;
          Serial.printf("Sunset = %d\r\n", sunset);
          for (JsonVariant item : list) {
            temp = (float)(item["main"]["temp"]);
            dt = (long long)(item["dt"]);
            if (start_time <= dt && stop_time >= dt) {
              clouds = (int)item["clouds"]["all"];
              Serial.println((const char*)item["dt_txt"]);
              Serial.printf("  dt = %d\r\n", dt);
              Serial.printf("  temp = %.2f°C\r\n", temp);
              if (temp_min == -1)
                temp_min = temp;
              else
                temp_min = min(temp_min, temp);
              if (sunset > dt) {
                Serial.printf("  clouds = %d\r\n", clouds);
                if (clouds < 10)
                  sun += 1.0;
                else if (clouds < 20)
                  sun += 0.5;
              }
            }
          }
        }
        Serial.printf("Forecasted min temperature = %.2f°C\r\n", temp_min);
        Serial.printf("Forecasted sun = %.2f\r\n", sun);
      }
    } else {
      Serial.printf("[HTTP] GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  } else {
    Serial.printf("[HTTP} Unable to connect\n");
  }
  return {temp_min, sun};
}


int get_override_cycles() {
  DateTime now = ntp.getEpochTime();
  if (override_end == 0) {
    return 0;
  } else if (now > override_end) {
    Serial.println("ERROR: override_end can't be negative");
    return 0;
  } else {
    // number of cycles left
    return ceil((float)(override_end - (long long)now.unixtime()) / (24 * 60 * 60));
  }
}


void get_state(char *state, Forecast forecast) {
  int target_hours = get_target_hours(forecast);
  sprintf(state, "forecast.temp=%.2f forecast.sun=%.2f, perceived_temp=%d, heater_state=%d, target_hours=%d, cached_forecast_hour=%d, override=%d, override_cycles=%d, override_end=%d, led=%d\r\n",
          forecast.temp,
          forecast.sun,
          get_perceived_temp(forecast),
          heater_state,
          target_hours,
          cached_forecast_hour,
          override,
          get_override_cycles(),
          override_end,
          led
         );
}


int get_perceived_temp(Forecast forecast) {
  return forecast.temp + sun_temp_coeficient * forecast.sun;
}


int get_target_hours(Forecast forecast) {
  int target_hours = 0;
  int perceived_temp = get_perceived_temp(forecast);
  for ( int i = 0; i < rows; ++i ) {
    if (perceived_temp < temp_hours[i][0]) {
      target_hours = temp_hours[i][1];
    }
  }
  return target_hours;
}


void read_udp_cmd() {
  char payload[255];  // buffer for incoming packets
  char *cmd;
  int steps;

  // receive incoming UDP packets
  int len = Udp.read(payload, 255);
  if (len > 0) {
    payload[len ] = 0;
  }
  Serial.printf("UDP packet contents: %s\n", payload);
  char *buffer = strdup(payload);

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  cmd = strsep(&buffer, ":");
  if ( cmd == NULL ) {
    Udp.write("INVALID FORMAT: expected cmd");
  } else {
    Serial.printf("  - Command: '%s'\n", cmd);
    if (strcmp(cmd, "status") == 0) {
      char state[255];
      get_state(state, cached_forecast);
      Udp.write(state);
    } else if ( strcmp(cmd, "set_override") == 0) {
      short override_value = atoi(strsep(&buffer, ":"));
      if (override_value == NULL) {
        Udp.write("set_override has invalid override value");
      } else {
        set_override(override_value);
        Udp.write("set_override ACK\r\n");
      }
    } else if ( strcmp(cmd, "set_override_cycles") == 0) {
      int override_cycles = atoi(strsep(&buffer, ":"));
      if ((override_cycles == NULL) || (override_cycles < 0)) {
        Udp.write("override_cycles has invalid override value");
      } else {
        set_override_cycles(override_cycles);
        Udp.write("set_override_cycles ACK\r\n");
      }
    } else if ( strcmp(cmd, "set_led") == 0) {
      short led_value = atoi(strsep(&buffer, ":"));
      if (led_value != 1 && led_value != 0) {
        Udp.write("set_led has invalid override value");
      } else {
        set_led(led_value);
        Udp.write("set_led ACK\r\n");
      }
    } else {
      Udp.write("Unkown command\r\n");
    }
  }
  Udp.endPacket();
}


void do_concurrent_tasks() {
  // stuff to do when moving stepper or blinking lights
  mqtt.loop();
  if (Udp.parsePacket()) {
    Serial.printf("Received packet from %s, port %d\n", Udp.remoteIP().toString().c_str(), Udp.remotePort());
    read_udp_cmd();
  }
  yield();
}

void set_override_cycles(int value) {
  Serial.printf("set_override_cycles(%d): ", value);
  if (value > 0) {
    DateTime now = ntp.getEpochTime();
    time_t stop_time;
    get_next_stop_time(now, &stop_time);
    --value;
    override_end = stop_time + (value * 24 * 60 * 60);
  } else if (value < 0) {
    Serial.printf("ERROR negative set_override_cycles not allowed");
    return;
  } else {
    override_end = 0;
  }
  int addr = 4;
  EEPROM.write(addr, (override_end >> 56) & 0xFF);
  EEPROM.write(addr + 1, (override_end >> 48) & 0xFF);
  EEPROM.write(addr + 2, (override_end >> 40) & 0xFF);
  EEPROM.write(addr + 3, (override_end >> 32) & 0xFF);
  EEPROM.write(addr + 4, (override_end >> 24) & 0xFF);
  EEPROM.write(addr + 5, (override_end >> 16) & 0xFF);
  EEPROM.write(addr + 6, (override_end >> 8) & 0xFF);
  EEPROM.write(addr + 7, override_end & 0xFF);
  if (EEPROM.commit()) {
    Serial.println("EEPROM set_override_cycles successfully committed");
  } else {
    Serial.println("ERROR! set_override_cycles EEPROM commit failed");
  }
  publish_mqtt_state(cached_forecast);
}

void set_override(short value) {
  Serial.printf("set_override(%d): ", value);
  if (value > 24 || value < 0) {
    Serial.println("ERROR: Invalid ser_override value");
    return;
  }
  override = value;
  // ESP 2 byte shorts
  int addr = 2;
  EEPROM.write(addr, override >> 8);
  EEPROM.write(addr + 1, override & 0xFF);
  if (EEPROM.commit()) {
    Serial.println("EEPROM set_override successfully committed");
  } else {
    Serial.println("ERROR! set_override EEPROM commit failed");
  }
  publish_mqtt_state(cached_forecast);
}


void set_led(short value) {
  Serial.printf("set_led(%d): ", value);
  if (value != 0 && value != 1) {
    Serial.println("ERROR: Invalid set_led value");
    return;
  }
  led = value;
  // ESP 2 byte shorts
  int addr = 12;
  EEPROM.write(addr, led >> 8);
  EEPROM.write(addr + 1, led & 0xFF);
  if (EEPROM.commit()) {
    Serial.println("EEPROM set_led successfully committed");
  } else {
    Serial.println("ERROR! set_led EEPROM commit failed");
  }
  publish_mqtt_state(cached_forecast);
}

void mqtt_callback(char* topic, byte * payload, unsigned int length) {
  payload[length] = NULL;
  Serial.printf("Message arrived [%s] '%s'\n", topic, (char*)payload);

  if (strcmp(topic, set_override_topic) == 0) {
    set_override(atoi((char*)payload));
  } else if (strcmp(topic, set_override_cycles_topic) == 0) {
    set_override_cycles(atoi((char*)payload));
  } else if (strcmp(topic, set_led_topic) == 0) {
    if (strcmp((char*)payload, "ON") == 0) {
      set_led(1);
    } else {
      set_led(0);
    }
  } else {
    Serial.println("Unknown topic");
  }
}

void send_mqtt_discovery_messages() {
  Serial.println("Publishing MQTT discovery ...");

  DynamicJsonDocument doc(1024);
  char buffer[512];
  size_t n;
  bool published;

  doc["stat_t"] = state_topic;
  doc["frc_upd"] = true;
  //doc["unit_of_meas"] = "";

  doc["name"] = "Target Hours";
  doc["val_tpl"] = "{{ value_json.target_hours|default(0) }}";
  doc["uniq_id"] = "main_heater-target_hours";
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/sensor/main_heater/target_hours/config", buffer, n);
  Serial.printf(" . target_hours discovery published (%d): %d\r\n", n, published);

  doc["name"] = "Cached Forecast Hour";
  doc["val_tpl"] = "{{ value_json.cached_forecast_hour|default(0) }}";
  doc["uniq_id"] = "main_heater-cached_forecast_hour";
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/sensor/main_heater/cached_forecast_hour/config", buffer, n);
  Serial.printf(" . cached_forecast_hour discovery published (%d): %d\r\n", n, published);

  doc["name"] = "Forecast Sun";
  doc["dev_cla"] = "illuminance";
  doc["val_tpl"] = "{{ value_json.forecast_sun|default(0) }}";
  doc["uniq_id"] = "main_heater-perceived_sun";
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/sensor/main_heater/forecast_sun/config", buffer, n);
  Serial.printf(" . forecast_sun discovery published (%d): %d\r\n", n, published);

  doc["name"] = "Forecast Temperature";
  doc["unit_of_meas"] = "°C";
  doc["dev_cla"] = "temperature";
  doc["val_tpl"] = "{{ value_json.forecast_temp|default(0)|round(1) }}";
  doc["uniq_id"] = "main_heater-forecast_temp";
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/sensor/main_heater/forecast_temp/config", buffer, n);
  Serial.printf(" . forecast_temp discovery published (%d): %d\r\n", n, published);

  doc["name"] = "Perceived Temp";
  doc["unit_of_meas"] = "°C";
  doc["dev_cla"] = "temperature";
  doc["val_tpl"] = "{{ value_json.perceived_temp|default(0) }}";
  doc["uniq_id"] = "main_heater-perceived_temp";
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/sensor/main_heater/perceived_temp/config", buffer, n);
  Serial.printf(" . perceived_temp discovery published (%d): %d\r\n", n, published);

  doc.remove("unit_of_meas");
  doc.remove("dev_cla");

  doc["name"] = "State";
  doc["dev_cla"] = "power";
  doc["val_tpl"] = "{{ value_json.state|default(0) }}";
  doc["uniq_id"] = "main_heater-state";
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/binary_sensor/main_heater/state/config", buffer, n);
  Serial.printf(" . statest_temp discovery published (%d): %d\r\n", n, published);

  doc.remove("dev_cla");
  doc.remove("frc_upd");

  doc["name"] = "LED";
  doc["val_tpl"] = "{{ value_json.led|default(0) }}";
  doc["uniq_id"] = "main_heater-led";
  doc["cmd_t"] = set_led_topic;
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/switch/main_heater/led/config", buffer, n);
  Serial.printf(" . led discovery published (%d): %d\r\n", n, published);

  doc["name"] = "Override Target Hours";
  doc["val_tpl"] = "{{ value_json.override|default(0) }}";
  doc["uniq_id"] = "main_heater-override";
  doc["min"] = -1;
  doc["max"] = 24;
  doc["cmd_t"] = set_override_topic;
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/number/main_heater/override/config", buffer, n);
  Serial.printf(" . override discovery published (%d): %d\r\n", n, published);

  doc["name"] = "Override Cycles";
  doc["val_tpl"] = "{{ value_json.override_cycles|default(0) }}";
  doc["uniq_id"] = "main_heater-override_cycles";
  doc["min"] = 0;
  doc["max"] = 30;
  doc["cmd_t"] = set_override_cycles_topic;
  n = serializeJson(doc, buffer);
  published = mqtt.publish("homeassistant/number/main_heater/override_cycles/config", buffer, n);
  Serial.printf(" . override_cycles discovery published (%d): %d\r\n", n, published);
}


int reconnect_wifi() {
  Serial.println("Reconnecting to WiFi...");
  WiFi.disconnect();
  WiFi.reconnect();
  for (int i = 0; i < 70; ++i) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("");
      return 0;
    }
    Serial.print("_");
    delay(500);
  }
  Serial.println("ERROR connecting to wifi");
  return -1;
}


int reconnect_mqtt() {
  if (WiFi.status() != WL_CONNECTED) {
    if (reconnect_wifi() == -1) {
      return -1;
    }
  }
  Serial.printf("MQTT connection status: %d\n", mqtt.state());
  if (mqtt.connect(mqtt_name, mqtt_user, mqtt_password)) {
    Serial.println("MQTT is now connected");
    send_mqtt_discovery_messages();
    mqtt.subscribe(set_override_topic);
    mqtt.subscribe(set_override_cycles_topic);
    mqtt.subscribe(set_led_topic);
    return 0;
  } else {
    Serial.printf("MQTT ERROR connecting to broker, failed with state %d\r\n", mqtt.state());
    return -1;
  }
}


void get_next_stop_time(DateTime now, time_t* stop_time) {
  tmElements_t tm;
  tm.Second = 0;
  tm.Minute = 0;
  tm.Hour = stop_hour;
  tm.Day = now.day();
  tm.Month = now.month();
  tm.Year = now.year() - 1970;
  time_t stop =  makeTime(tm);
  if (now > stop) {
    now = now + 86400UL;
    tm.Day = now.day();
    tm.Month = now.month();
    tm.Year = now.year() - 1970;
  }
  *stop_time = makeTime(tm);
}


int publish_mqtt_state(Forecast forecast) {
  Serial.println("Publishing MQTT state ...");
  if (!mqtt.connected()) {
    if (reconnect_mqtt() != 0) {
      return -1;
    }
  }
  DynamicJsonDocument doc(1024);
  char buffer[256];
  doc["forecast_temp"] = forecast.temp;
  doc["forecast_sun"] = forecast.sun;
  doc["perceived_temp"] = get_perceived_temp(forecast);
  if (heater_state == 1)
    doc["state"] = "ON";
  else if (heater_state == 0)
    doc["state"] = "OFF";
  else
    doc["state"] = "UNKOWN";
  doc["target_hours"] = get_target_hours(forecast);
  doc["cached_forecast_hour"] = cached_forecast_hour;
  doc["override"] = override;
  doc["override_cycles"] = get_override_cycles();
  if (led == 1)
    doc["led"] = "ON";
  else
    doc["led"] = "OFF";

  size_t n = serializeJson(doc, buffer);

  bool published = mqtt.publish(state_topic, buffer, n);
  Serial.printf(" . State published (%d): %d\r\n", n, published);
  return published;
}

void setup() {
  pinMode(HOMING_PIN, INPUT_PULLUP);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // booting colors
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);

  Serial.begin(115200);

  Serial.printf("\r\n\r\nConnecting to %s ", ssid);
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */

  WiFi.mode(WIFI_STA);
  WiFi.hostname(device_name);
  WiFi.config(ip, dns, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  Serial.print("Syncing with NTP ");
  while (! ntp.update()) {
    Serial.print("-");
  }
  Serial.println("");

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setBufferSize(512);
  mqtt.setCallback(mqtt_callback);
  reconnect_mqtt();

  Serial.print("Reading stored override from EEPROM: ");
  EEPROM.begin(2 + 2 + 8 + 2); // magic number, override, override_end
  int addr = 0;
  short actual_magic_number = (EEPROM.read(addr) << 8) + EEPROM.read(addr + 1);
  if (eeprom_magic_number !=  actual_magic_number) {
    Serial.printf("Store is empty %d != %d, setting default values...\n", actual_magic_number, eeprom_magic_number);
    set_override(override);
    set_override_cycles(override_end);
    set_led(led);
    EEPROM.write(addr, eeprom_magic_number >> 8);
    EEPROM.write(addr + 1, eeprom_magic_number & 0xFF);
    EEPROM.commit();
    Serial.println((EEPROM.read(addr) << 8) + EEPROM.read(addr + 1));

  } else {
    addr = 2;
    override = ((int)EEPROM.read(addr) << 8) + (int)EEPROM.read(addr + 1);
    addr = 4;
    override_end = ((long long)EEPROM.read(addr) << 56) +
                   ((long long)EEPROM.read(addr + 1) << 48) +
                   ((long long)EEPROM.read(addr + 2) << 40) +
                   ((long long)EEPROM.read(addr + 3) << 32) +
                   ((long long)EEPROM.read(addr + 4) << 24) +
                   ((long long)EEPROM.read(addr + 5) << 16) +
                   ((long long)EEPROM.read(addr + 6) << 8) +
                   (long long)EEPROM.read(addr + 7);
    addr = 12;
    led = (EEPROM.read(addr) << 8) + EEPROM.read(addr + 1);
    Serial.printf("Read override=%d, override_cycles=%d, led=%d, override_end=", override, get_override_cycles(), led);
    Serial.print(override_end);
    Serial.println("");
  }
}


void loop() {
  DateTime now = ntp.getEpochTime();
  Serial.println(String("Current time:\t") + now.timestamp(DateTime::TIMESTAMP_FULL));
  Forecast forecast = get_forecast(now);
  int target_hours = get_target_hours(forecast);
  int effective_target_hours = target_hours;
  long long stop_time;
  get_next_stop_time(now, &stop_time);

  // apply override
  if (override != -1) {
    if ((override_end == 0) || (now < override_end)) {
      effective_target_hours = override;
    } else if (override_end > 0 && now > override_end) {
      set_override(-1);
      set_override_cycles(0);
    }
  }

  // set heater state
  if ( ((stop_time - (effective_target_hours * 60 * 60)) < now.unixtime()) && (now.unixtime() < stop_time) ) {
    if (heater_state != 1)
      heater_on();
  } else if (heater_state != 0) {
    heater_off();
  }

  publish_mqtt_state(forecast);
  char state[255];
  get_state(state, forecast);
  Serial.print(state);
  // sleep for 1 minute but maybe blink LED
  for (int i = 0; i < 2 * 60; ++i) {
    if (led == 1) {
      if (override != -1) {
        digitalWrite(RED, HIGH);
        digitalWrite(GREEN, HIGH);
        digitalWrite(BLUE, HIGH);
      } else if (forecast.temp == -1 || heater_state == -1) {
        digitalWrite(RED, HIGH);
        digitalWrite(GREEN, LOW);
        digitalWrite(BLUE, LOW);
      } else if (target_hours > moderate_hours) {
        digitalWrite(RED, HIGH);
        digitalWrite(GREEN, LOW);
        digitalWrite(BLUE, HIGH);
      } else if (target_hours > 0) {
        digitalWrite(RED, LOW);
        digitalWrite(GREEN, LOW);
        digitalWrite(BLUE, HIGH);
      } else {
        digitalWrite(RED, LOW);
        digitalWrite(GREEN, HIGH);
        digitalWrite(BLUE, LOW);
      }
      if (heater_state == 1 && i % 2) {
        //blink
        digitalWrite(RED, LOW);
        digitalWrite(GREEN, LOW);
        digitalWrite(BLUE, LOW);
      }
    } else {
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, LOW);
    }
    do_concurrent_tasks();
    delay(500);
  }
  if (led == 1) {
    // looping colors
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, LOW);
  }
}
