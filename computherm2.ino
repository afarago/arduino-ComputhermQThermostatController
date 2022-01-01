#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
//#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <NTPClient.h>
#include <ArduinoOTA.h>
#include <TimeLib.h>
#include <LittleFS.h>
#include <computhermrf.h>
#include "arduino_secrets.h"

#define PROJECT_NAME "WIFITO868GW"

/*
 * ComputhermQThermostat controller
 *  based on the work of denxhun, flogi, nistvan86
 *  by Attila Farago, 2021
 *
 *  This is an Arduino D1 Mini project which allows you to control a Computherm Q7RF/Q8RF receiver 
 *  using a RFM117W transmitter and RFM217W receiver module.
 */

////---------- ComputhermQThermostat class forward declaration ----------//

class ComputhermQThermostat {
  public:
    ComputhermQThermostat();
    void setup(const char* device_sid, bool readonly, int idx_channel, const char* description);
    void update();
    void on_pairing();
    void set_state_update_rf(bool state);
    void set_state_update_mqtt(bool state);
    const char* get_device_id();
    bool get_readonly();
    bool get_state();
    const char* get_description();

  public:
    String getjson();

  //!!private:
  public:
    unsigned int idx_channel_ = 0;
    unsigned long last_msg_time_ = 0;
    unsigned long last_display_time_ = 0;

    bool state_ = false;
    bool initialized_ = false;
    uint8_t pending_msg_ = 0; // 0 = MSG_NONE
    unsigned long last_change_timestamp = 0;

    bool config_readonly_ = true;
    const char* config_device_sid_;
    const char* config_device_description_;
    uint16_t config_resend_interval_ = 60000;

  private:
    void send_msg(uint8_t msg);
    bool update_state(bool newstate);
    void display_blink_led();
};

////---------- ARDUINO MAIN ----------//

void setup() {
  // put your setup code here, to run once:
  // Setup PINs
  pinMode(BUILTIN_LED, OUTPUT);
  LED(true);

  // Set software serial baud to 115200;
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Computherm radio detector and MQTT handler (wifito868gw) by afarago, based on the work of denxhun, flogi, nistvan86."));

  // LittleFs
  littlefs_setup();

  // setup and connect RF handler
  rfhandler_setup();

  // setup DHT22 handler
  dht22handler_setup();

  // connecting to a WiFi network
  wifihandler_setup();
  wifihandler_connect();
  //wifihandler_wait_connected();

  //connecting to a mqtt broker
  mqtthandler_setup(String(WiFi.macAddress()));
  //mqtthandler_ensure_connected();

  // OTA handler
  otahandler_setup();

  // setup NTP
  ntphandler_setup();

  // setup thermostats
  computhermqhandler_setup();

  // finish setup and shut down LED
  LED(false);
}

void loop() {
  if (!otahandler_isupdating()) {
    wifihandler_loop();
    mqtthandler_loop();
    ntphandler_loop();
    otahandler_loop();
    LEDhandler_loop();
    
    if (mqtthandler_connected()) {
      rfhandler_loop();
      computhermqhandler_loop();
      dht22handler_loop();
    }
  }
//  delay(100);
}

////---------- UTILITIES ----------//
void LED(bool on) {
  digitalWrite(BUILTIN_LED, on ? LOW : HIGH);
}

const float ledhb_gamma = 0.14; // affects the width of peak (more or less darkness)
const float ledhb_beta = 0.5; // shifts the gaussian to be symmetric
void LED_heartbeat(int duration = 750, int max_brightness = 255) {
  const float led_smoothness_pts = duration/5;
  for (int ii=0;ii<led_smoothness_pts;ii++){
    float pwm_val = (255-max_brightness) + (float)max_brightness*(1-exp(-(pow(((ii/led_smoothness_pts)-ledhb_beta)/ledhb_gamma,2.0))/2.0));
    analogWrite(BUILTIN_LED,int(pwm_val));
    delay(5);
  }
  digitalWrite(BUILTIN_LED, HIGH);
}

void LED_msg_blink(int count) {
  // one long then X short blinks
  LED(true); delay(600); LED(false); delay(400); 
  for (int i=0; i<count; i++) {
    LED(true); delay(200); LED(false); delay(200);
  }
}

unsigned long last_heartbeat_display_time = 0;
void LEDhandler_loop() {
  // heartbeat --> on error/disconnect --> continous fast pulse 
  if (!mqtthandler_connected()) { 
    LED_heartbeat(350, 50); 
  }
  // heartbeat --> normal operations --> heartbet every minute
  else if ( calculate_diff(millis(), last_heartbeat_display_time) > 60000 ) {
    last_heartbeat_display_time = millis();
    LED_heartbeat(750, 50);
  }
}

unsigned long calculate_diff(long now, long last_update) {  
  unsigned long diff = 0UL;

  if (last_update > now) {
    // millis() overflows every ~50 days
    diff = (ULONG_MAX - last_update) + now;
  } else {
    diff = now - last_update;
  }
  return diff;
}

const char* ONOFF(bool state) {
  return state ? "ON" : "OFF";
}

bool isONOFF(const char *state) {
  return (strcmp(state,"ON")==0) ? 1 : 0;
}

const char* TRUEFALSE(bool value) {
  return value ? "true" : "false";
}

//////---------- LittleFS handler ----------//
bool littlefs_setup() {
  bool result = LittleFS.begin();
  return result;
}

//////---------- OTA handler ----------//
bool ota_updating = false;
bool otahandler_isupdating() {
  return ota_updating;
}

void otahandler_setup() {
  ArduinoOTA.setHostname(PROJECT_NAME);
//  ArduinoOTA.setPassword(XXX);
  ArduinoOTA.onStart([]() {
    Serial.println(F("OTA Start"));
    LittleFS.end();
    ota_updating = true;
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("\nOTA End"));
    //ESP.restart();
    //WDT - After a serial upload, the ESP must be manually reset before an OTA can work (actually, I think before a software reset as well).
    ota_updating = false;
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int progresspc = progress / (total / 100);
    Serial.printf("OTA Progress: %u%%\r", progresspc);
    LED(progresspc % 2 == 0);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
//    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}
  
void otahandler_loop() {
  ArduinoOTA.handle();
}

//////---------- NTP handler ----------//
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
const long utcOffsetInHour = 1;
const long utcOffsetInSeconds = utcOffsetInHour*3600;
unsigned long ntphandler_startupTime = 0;

void ntphandler_setup() {
  timeClient.setTimeOffset(utcOffsetInSeconds);
  timeClient.begin();
}

void ntphandler_loop() {
  timeClient.update();
  
  // start init if NTP was never inited
  if (!ntphandler_startupTime) {
    // wait4validtime
    Serial.println(F("Waiting for NTP"));
    if (!ntphandler_isvalid()) {
      delay(100);
      timeClient.update();
    } else {
      // NTP init done - first reading is recorded as startup time
      ntphandler_startupTime = timeClient.getEpochTime();
    }
  }
}

bool ntphandler_isvalid() {
  return timeClient.getEpochTime() > utcOffsetInSeconds*2;
}

String ntphandler_formatTime(long unsigned ts) {
  char lastchange_s[32]; //!!
  sprintf(lastchange_s, "%02d-%02d-%02dT%02d:%02d:%02d+%02d00", 
    year(ts), month(ts), day(ts), 
    hour(ts), minute(ts), second(ts),
    utcOffsetInHour);

  return String(lastchange_s);
}

String ntphandler_getFormattedTime() {
  //return timeClient.getFormattedTime();
  long unsigned ts = ntphandler_getEpochTime();

  return(ntphandler_formatTime(ts));
}

long unsigned ntphandler_getEpochTime () {
  return timeClient.getEpochTime();
}

////---------- DHT22 handler ----------//
DHTesp dht;
static const long dht22handler_update_delay = 60000; //update temp and humidity every 60sec
unsigned long dht22handler_last_update = dht22handler_update_delay; //to trigger instant update

void dht22handler_setup() {
  dht.setup(D3, DHTesp::DHT22);
}

void dht22handler_loop() {
  long now = millis();
  if ( calculate_diff(now, dht22handler_last_update) > dht22handler_update_delay ) {
    dht22handler_last_update = now;

    String payload = dht22handler_getjson();
    mqtthandler_send_sensor_update(payload);
  }
}

String dht22handler_getjson() {
  float h = dht.getHumidity();
  float t = dht.getTemperature();
  String payload = 
    "{\"temp\":" + String(t,2) + 
    ",\"humidity\":" + String(h,2) +
    "}";
  return payload;
}

////---------- WiFi handler ----------//
static const char *wifihandler_ssid = SECRET_WIFI_SSID;
static const char *wifihandler_password = SECRET_WIFI_PASSWORD;
WiFiClient wifihandler_espClient;
WiFiEventHandler gotIpEventHandler; //, disconnectedEventHandler;
ESP8266WebServer server(80);   //instantiate server at port 80 (http port)

//const char* wl_status_to_string(wl_status_t status) {
//  switch (status) {
//    case WL_NO_SHIELD: return "WL_NO_SHIELD";
//    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
//    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
//    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
//    case WL_CONNECTED: return "WL_CONNECTED";
//    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
//    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
//    case WL_DISCONNECTED: return "WL_DISCONNECTED";
//  }
//}

void wifihandler_setup() {
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event)
  {
    Serial.printf("WIFI connected, IP: %s\n", WiFi.localIP().toString().c_str());
  });

//  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
//  {
//    Serial.println("WIFI disconnected");
//  });

//  if(!MDNS.begin(PROJECT_NAME)) {
//    Serial.println("Error starting mDNS");
//  }
//  MDNS.addService("http", "tcp", 80);
  
  server.on("/", wifihandler_handleroot);
  server.on("/status", wifihandler_handleroot);
  server.on("/ping", [](){
    server.send(200, "application/json", "OK: "+ ntphandler_getFormattedTime());
    LED_heartbeat();
  });
  server.on("/restart", [](){
    server.send(200, "application/json", "OK");
    ESP.restart();
  });
  server.begin();
}

void wifihandler_handleroot() {
 String payload = 
    "{\"status\":" + mqtthandler_getjson() + 
    ",\"sensor\":" + dht22handler_getjson() + 
    ",\"computherm\":" + computhermqhandler_getjson() + 
    "}";
  server.send(200, "application/json", payload);
}

bool wifihandler_connect() {
  // wake up wifi if needed
  WiFi.forceSleepWake();
  delay(1);

  // connect wifi
  WiFi.hostname(PROJECT_NAME);
  WiFi.begin(wifihandler_ssid, wifihandler_password);
  Serial.println(F("WIFI connecting..."));

  return wifihandler_connected();
}

void wifihandler_wait_connected() {
  while (!wifihandler_connected()) { 
    delay(500); 
  }
}

bool wifihandler_connected() {
  return WiFi.status() == WL_CONNECTED;
}

void wifihandler_ensure_connected() {
  if (WiFi.status() != WL_CONNECTED) {
    wifihandler_connect();
    wifihandler_wait_connected();
  }
}

void wifihandler_disconnect() {
  WiFi.disconnect();
  WiFi.forceSleepBegin();
}

void wifihandler_loop() {
  server.handleClient();
//  MDNS.update();
}

////---------- MQTT PubSubHandler ----------//
static const char mqtt_broker[] = SECRET_MQTT_SERVER;
static const char mqtt_username[] = SECRET_MQTT_USERNAME;                 
static const char mqtt_password[] = SECRET_MQTT_PASSWORD;
static const int mqtt_port = SECRET_MQTT_PORT;

PubSubClient mqtthandler_pubsubclient(wifihandler_espClient);
String mqtthandler_client_id;
unsigned long mqtthandler_lastconnected = 0;
unsigned long mqtthandler_lastconnectattempt = 0;
bool mqtthandler_manual_shutdown_sent;
static const long mqtthandler_connection_timeout_rf_shutdown = 60000;
static const long mqtthandler_connection_timeout_soft_reboot = 3600000; // 1h
static const long mqtthandler_update_delay = 60000; //update base status every 60sec
unsigned long mqtthandler_last_update = mqtthandler_update_delay; //to trigger instant update

static const char mqtt_base_t[] = "wifito868gw/";
static String mqtt_status_topic = "/status";
static String mqtt_sensor_topic = "/sensor/status";
static String mqtt_computherm_topic = "/computherm";
static String mqtt_availability_topic = "/availability";
static const char mqtt_availability_message_offline[] = "offline";
static const unsigned char mqtt_availability_message_online[] = "online";
static const char mqtt_unknown_subtopic_w_slash[] = "unknown/";

static const char mqtt_wildcard_wildcard[] = "/+";
static const char mqtt_status_subtopic[] = "/status";
static const char mqtt_control_subtopic[] = "/control";
static const char mqtt_pair_subtopic[] = "/pair";

void mqtthandler_setup(String mac_address) {
  mqtthandler_pubsubclient.setServer(mqtt_broker, mqtt_port);
  mqtthandler_client_id = "esp8266-wifito868gw-" + mac_address;
  mqtt_status_topic = String(mqtt_base_t) + mac_address + mqtt_status_topic;
  mqtt_sensor_topic = String(mqtt_base_t) + mac_address + mqtt_sensor_topic;
  mqtt_computherm_topic = String(mqtt_base_t) + mac_address + mqtt_computherm_topic;
  mqtt_availability_topic = String(mqtt_base_t) + mac_address + mqtt_availability_topic;
}

void mqtthandler_ensure_connected() {
  if (!mqtthandler_connected()) {
    wifihandler_ensure_connected();
    mqtthandler_connect_wait();
  }
}

bool mqtthandler_connected() {
  return mqtthandler_pubsubclient.connected();
}

void mqtthandler_connect_wait() {
  while (!mqtthandler_pubsubclient.connected()) {
    if (!mqtthandler_connect()) {
      Serial.printf("MQTT failed with state %d\n", mqtthandler_pubsubclient.state());
      delay(1000);
    }
  }
}

bool mqtthandler_connect() {
  Serial.printf("MQTT connecting with client %s...\n", mqtthandler_client_id.c_str());
  if (mqtthandler_pubsubclient.connect(mqtthandler_client_id.c_str(), mqtt_username, mqtt_password, mqtt_availability_topic.c_str(), 0, false, mqtt_availability_message_offline)) {

    // set callback
    mqtthandler_pubsubclient.setCallback(mqtthandler_callback);
  
    // subscribe
    String topicmod = mqtt_computherm_topic; topicmod += mqtt_wildcard_wildcard; topicmod += mqtt_control_subtopic;    
    mqtthandler_pubsubclient.subscribe(topicmod.c_str());
    topicmod = mqtt_computherm_topic; topicmod += mqtt_wildcard_wildcard; topicmod += mqtt_pair_subtopic;
    mqtthandler_pubsubclient.subscribe(topicmod.c_str());

    // send availability message
    mqtthandler_pubsubclient.publish(mqtt_availability_topic.c_str(), mqtt_availability_message_online, sizeof(mqtt_availability_message_online)-1, true);

    // set flgs
    mqtthandler_manual_shutdown_sent = false;

    return true;
  } else {
    return false;
  }
}

void mqtthandler_sendstatus(String ttid, String payload) {
  if (mqtthandler_connected()) {
    String topicmod = mqtt_computherm_topic;
    topicmod += "/";
    topicmod += ttid;
    topicmod += mqtt_status_subtopic;
  
    Serial.printf("MQTT send topic: %s, Message: %s\n", topicmod.c_str(), payload.c_str());
    //mqtthandler_ensure_connected();
    mqtthandler_pubsubclient.publish(topicmod.c_str(), (const uint8_t*)payload.c_str(), payload.length(), true);
  }
}

void mqtthandler_callback(char *topic, byte *payload, unsigned int length) {
  // split ttid from: wifito868gw/computherm/8A000/control
  String topicstr = topic;
  if (!topicstr.startsWith(mqtt_computherm_topic.c_str())) return;

  int delimiter_index = topicstr.indexOf("/", mqtt_computherm_topic.length()+1);
  if (delimiter_index<0) return;
//  Serial.print("[D]"); Serial.println(mqtt_computherm_topic);
//  Serial.print("[D]"); Serial.println(mqtt_computherm_topic.length());
//  Serial.print("[D]"); Serial.println(delimiter_index);
  
  String ttid = topicstr.substring(mqtt_computherm_topic.length()+1, delimiter_index);
  String actionverb = topicstr.substring(delimiter_index); // include the starting "/" char as well

  // payload:
  payload[length] = '\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;
  Serial.printf("MQTT Message arrived in topic:[%s] ttid:[%s] action:[%s] message: [%s]\n", 
    topicstr.c_str(), ttid.c_str(), actionverb.c_str(), message.c_str());

  // check if instance exists and if is not readonly otherwise skip
  ComputhermQThermostat *thermo = computhermqhandler_findbyid(ttid.c_str());

  if (!thermo) {
    Serial.println(F("MQTT unregistered thermostat - skipping control command"));
  
  } else {  
    // update computherm by RF sender:
    if (actionverb == mqtt_control_subtopic) {
      if (thermo->get_readonly()) {
        Serial.println(F("MQTT read-only thermostat - skipping control command"));
        
      } else {
        bool vez = isONOFF(message.c_str());
        
        // update through RF
        thermo->set_state_update_rf(vez);
        
        // update base topic / state --> //TODO: set state and send mqtt update - later, after setting is done??
        thermo->set_state_update_mqtt(vez);
      }
      
    } else if (actionverb == mqtt_pair_subtopic) {
      thermo->on_pairing();
    }
  }
}

void mqtthandler_send_sensor_update(String payload) {
  if (mqtthandler_connected()) {
    Serial.printf("MQTT send sensor update: %s, Message: %s\n", mqtt_sensor_topic.c_str(), payload.c_str());
    mqtthandler_pubsubclient.publish(mqtt_sensor_topic.c_str(), (const uint8_t*)payload.c_str(), payload.length(), false);
  }
}

void mqtthandler_send_status_update(String payload) {
  if (mqtthandler_connected()) {
    Serial.printf("MQTT send status update: %s, Message: %s\n", mqtt_status_topic.c_str(), payload.c_str());
    mqtthandler_pubsubclient.publish(mqtt_status_topic.c_str(), (const uint8_t*)payload.c_str(), payload.length(), false);
  }
}

String mqtthandler_getjson() {
  String payload = 
    String("{\"uptime\":") + (int)(millis()/1000) + 
    ",\"time\":\"" + ntphandler_getFormattedTime() + "\""
    ",\"unixtime\":\"" + ntphandler_getEpochTime() + "\""
    ",\"freeheap\":\"" + String(ESP.getFreeHeap()) + "\""
    "}";
  return payload;
}

void mqtthandler_loop() {
  long now = millis();
  if (mqtthandler_connected()) {
    mqtthandler_pubsubclient.loop();
    mqtthandler_lastconnected = now;

    if ( calculate_diff(now, mqtthandler_last_update) > mqtthandler_update_delay ) {
      mqtthandler_last_update = now;
  
      String payload = mqtthandler_getjson();
      mqtthandler_send_status_update(payload);
    }

  } else {
    // reconnect in a nonblocking way
    if (calculate_diff(now, mqtthandler_lastconnectattempt) > 10000) {
      // attempt to reconnect every 10 sec
      mqtthandler_lastconnectattempt = now;

      if (wifihandler_connected()) {
        if (mqtthandler_connect()) {
          mqtthandler_lastconnectattempt = 0;
          return;
        }
      }
      else {
        if (wifihandler_connect()) {
          if (mqtthandler_connect()) {
            mqtthandler_lastconnectattempt = 0;
            return;
          }
        }
      }
    }

    // if no MQTT connection in like 60 sec -- turn off any manual switches
    if ((calculate_diff(now, mqtthandler_lastconnected) > mqtthandler_connection_timeout_rf_shutdown) && 
          !mqtthandler_manual_shutdown_sent) {
      Serial.println(F("MQTT disconnection timeout - manually shutdown all non readonly thermostats"));
      mqtthandler_manual_shutdown_sent = true;
      computhermqhandler_shutdown_all_non_readonly();
    }
    
    // if no MQTT connection in like 1 h -- perform soft reset to retry wifi/mqtt
    if (calculate_diff(now, mqtthandler_lastconnected) > mqtthandler_connection_timeout_soft_reboot) {
      ESP.restart();
    }      
  }
}

////---------- rfhandler ----------//
ComputhermRF rfhandler_rf = ComputhermRF(D1, D2);
void rfhandler_setup() {
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  rfhandler_rf.startReceiver();
  Serial.println(F("Computherm receiver started."));
}

void rfhandler_loop() {
  if (rfhandler_rf.isDataAvailable()) {
//    last_RFMessage_millis = currentMillis; //TODO: apply and trigger on last change only
    computhermMessage msg = rfhandler_rf.getData();
    Serial.printf("RF New message caught. Address: %s command: %s\n", msg.address, msg.command);

    ComputhermQThermostat *thermo = computhermqhandler_findbyid(msg.address.c_str());
    if (!thermo) {
      Serial.println(F("MQTT unregistered thermostat - still registering data over MQTT"));
      //Serial.println(F("MQTT unregistered thermostat - skipping"));
      mqtthandler_sendstatus(String(mqtt_unknown_subtopic_w_slash)+String(msg.address), ntphandler_getFormattedTime());
    } else {
      thermo->set_state_update_mqtt(isONOFF(msg.command.c_str()));
    }
  }
}

////---------- ComputhermQThermostat ----------//

static const int rf_repeat_count = 4;

static const uint8_t MSG_NONE = 0;
static const uint8_t MSG_HEAT_ON = 1;
static const uint8_t MSG_HEAT_OFF = 2;
static const uint8_t MSG_PAIR = 3;

ComputhermQThermostat::ComputhermQThermostat() {
  
}

const char* ComputhermQThermostat::get_device_id() {
  return this->config_device_sid_;
}

bool ComputhermQThermostat::get_readonly() {
  return this->config_readonly_;
}

bool ComputhermQThermostat::get_state() {
  return this->state_;
}
const char *ComputhermQThermostat::get_description() {
  return this->config_device_description_;
}

void ComputhermQThermostat::setup(const char* device_sid, bool readonly, int idx_channel, const char* description) {
  this->config_device_sid_ = device_sid;
  this->config_device_description_ = description;
  this->config_readonly_ = readonly;
  this->idx_channel_ = idx_channel;
  // Revert switch to off state
  this->state_ = false;

  // Read from LittleFs
  String filename = "/"+String(this->config_device_sid_);
  File devfile = LittleFS.open(filename, "r");
  if (devfile){
    String lastchanges = devfile.readString(); //readStringUntil, file.readbytes(buffer, length),  file.write(buffer, length) 
    last_change_timestamp = atol(lastchanges.c_str());
    devfile.close();
  }

  // update through RF - will be auto updated on repeat for OFF, if there is no other control
  if (!this->config_readonly_) {
    // send current state (OFF) via RF
    this->set_state_update_rf(this->state_);
  }
}

void ComputhermQThermostat::update() {
  if (!this->initialized_) {
    this->initialized_ = true;
    // initialize mqtt state
    this->set_state_update_mqtt(this->state_);
  }

  if (this->pending_msg_ != MSG_NONE) {
    // Send prioritized message
    this->send_msg(this->pending_msg_);
    this->pending_msg_ = MSG_NONE;
    
  } else if (!this->config_readonly_) {
    // resend message over RF - if thermostat is not read only
    unsigned long now = millis();

    // Check if we have to resend current state by now
    if ( calculate_diff(now, this->last_msg_time_) > this->config_resend_interval_ ) {
      uint8_t msg = this->state_ ? MSG_HEAT_ON : MSG_HEAT_OFF;
      this->send_msg(msg);
    }

    // if thermostat is ON, blink LED to show state X times
    if ( calculate_diff(now, last_display_time_) > 10000 ) {
      if (this->state_) display_blink_led();
      last_display_time_ = now;
    }
  }
}

bool ComputhermQThermostat::update_state(bool newstate) {
  if (this->state_ == newstate) return false;

  this->state_ = newstate;
  last_change_timestamp = ntphandler_getEpochTime();

  // Write to LittleFs
  String filename = "/"+String(this->config_device_sid_);
  File devfile = LittleFS.open(filename, "w");
  if (devfile){
    devfile.print(last_change_timestamp);
    devfile.close();
  }

  // if thermostat is ON, blink LED to show state X times
  if (newstate) this->display_blink_led();

  return true;
}

void ComputhermQThermostat::display_blink_led() {
  LED_msg_blink(this->idx_channel_+1);
}

void ComputhermQThermostat::send_msg(uint8_t msg) {
  if (msg == MSG_NONE) return;

  Serial.printf("RF Sending message: 0x%02x for %d:%s\n", msg, this->idx_channel_, this->config_device_sid_);

  for (int i = 0; i < rf_repeat_count; i++) {
    switch (msg) {
      case MSG_HEAT_ON:
        rfhandler_rf.sendMessage(this->config_device_sid_, true);
        break;
      case MSG_HEAT_OFF:
        rfhandler_rf.sendMessage(this->config_device_sid_, false);
        break;
      case MSG_PAIR:
        rfhandler_rf.pairAddress(this->config_device_sid_);
        break;
    }
  }

  this->last_msg_time_ = millis();
}

void ComputhermQThermostat::set_state_update_mqtt(bool state) {
  update_state(state);
  
  String command = this->getjson();
  mqtthandler_sendstatus(config_device_sid_, command); 
}

void ComputhermQThermostat::set_state_update_rf(bool state) {
  update_state(state);

  this->pending_msg_ = state ? MSG_HEAT_ON : MSG_HEAT_OFF;
}

void ComputhermQThermostat::on_pairing() {
  // enqueue pairing
  this->pending_msg_ = MSG_PAIR;
}

String ComputhermQThermostat::getjson() {
  String result =
    String("{\"state\":\"") + ONOFF(this->get_state()) + "\"" +
    ",\"description\":\"" + String(this->get_description()) + "\"" +
    ",\"last_change\":\"" + String(ntphandler_formatTime(last_change_timestamp)) + "\"" +
    ",\"readonly\":" + TRUEFALSE(this->get_readonly()) +
    "}";

  return result;
}

////---------- computhermqhandler ----------//
static const unsigned int thermo_count = SECRET_THERMO_COUNT;
static const char thermo_id[thermo_count][6] = SECRET_THERMO_IDS;
static const char *thermo_descriptions[thermo_count] = SECRET_THERMO_DESCS;
static const bool thermo_readonly[thermo_count] = SECRET_THERMO_READONLY;
ComputhermQThermostat thermos[4];

void computhermqhandler_setup() {
  for (int i = 0; i < thermo_count; i++) {
    thermos[i].setup(thermo_id[i], thermo_readonly[i], i, thermo_descriptions[i]);
  }
}

void computhermqhandler_loop() {
  for (int i = 0; i < thermo_count; i++) {
    thermos[i].update();
  }
}

ComputhermQThermostat* computhermqhandler_findbyid(const char* device_sid) {
  for (int i = 0; i < thermo_count; i++) {
    if (strcmp(thermos[i].get_device_id(), device_sid) == 0) {
      return &thermos[i];
    }
  }
  return NULL;
}

void computhermqhandler_shutdown_all_non_readonly() {
  for (int i = 0; i < thermo_count; i++) {
    if (!thermos[i].get_readonly()) {
      thermos[i].set_state_update_rf(false);
    }
  }
}

String computhermqhandler_getjson() {
  String result = "{\"thermostats\": [";
  for (int i = 0; i < thermo_count; i++) {
    if (i>0) result += ",";
    result += "{\"" + String(thermos[i].get_device_id()) + "\":" + thermos[i].getjson() + "}";
  }
  result += "]}";
  return result;
}
