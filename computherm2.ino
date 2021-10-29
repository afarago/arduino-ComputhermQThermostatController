#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <computhermrf.h>
#include "arduino_secrets.h"

/*
 * ComputhermQThermostat controller
 *  based on the work of denxhun, flogi, nistvan86
 *  by Attila Farago, 2021
 *
 *  This is an Arduino D1 Mini project that allows reading multiple Computherm Q thermostats and 
 *  furthermore create, control virtual thermostats.
 */

////---------- ComputhermQThermostat class forward declaration ----------//

class ComputhermQThermostat {
  public:
    ComputhermQThermostat();
    void setup(String device_sid, bool readonly, int idx_channel);
    void update();
    void on_pairing();
    String device_id();
    void set_state_update_rf(bool state);
    void set_state_update_mqtt(bool state);
    bool readonly();

  public:
    void set_config_device_sid(String value);

  private:
    unsigned int idx_channel_ = 0;
    unsigned long last_msg_time_ = 0;
    unsigned long last_display_time_ = 0;
    bool state_ = false;
    bool config_readonly_ = true;
    String config_device_sid_;
    uint16_t config_resend_interval_ = 60000;

    uint8_t pending_msg_ = 0;
    bool initialized_ = false;

  private:
    void send_msg(uint8_t msg);
    void display_blink_led();
    void initialize_mqtt_config();
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
  Serial.println("Computherm radio detector and MQTT handler (wifito868gw) by afarago, based on the work of denxhun, flogi, nistvan86.");

  // setup and connect RF handler
  rfhandler_setup();

  // setup DHT22 handler
  dht22handler_setup();

  // connecting to a WiFi network
  wifihandler_setup();
  wifihandler_connect();
  wifihandler_wait_connected();

  //connecting to a mqtt broker
  mqtthandler_setup(String(WiFi.macAddress()));
  mqtthandler_ensure_connected();

  // setup thermostats
  computhermqhandler_setup();

  // finish setup and shut down LED
  LED(false);
}

void loop() {
  mqtthandler_loop();
  rfhandler_loop();
  computhermqhandler_loop();

  delay(100);

  dht22handler_loop();
}

////---------- UTILITIES ----------//
void LED(bool on) {
  digitalWrite(BUILTIN_LED, on ? LOW : HIGH);
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
    float h = dht.getHumidity();
    float t = dht.getTemperature();

    String payload = "{\"temp\": " + String(t,2) + ", \"humidity\": " + String(h,2) + "}";
    mqtthandler_send_sensor_update(payload);
  }
}

////---------- WiFi handler ----------//
static const char *wifihandler_ssid = SECRET_WIFI_SSID;
static const char *wifihandler_password = SECRET_WIFI_PASSWORD;
WiFiClient wifihandler_espClient;
WiFiEventHandler gotIpEventHandler; //, disconnectedEventHandler;

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
}

void wifihandler_connect() {
  // wake up wifi if needed
  WiFi.forceSleepWake();
  delay(1);

  // connect wifi
  WiFi.begin(wifihandler_ssid, wifihandler_password);
  Serial.println("WIFI connecting...");
}

void wifihandler_wait_connected() {
  while (WiFi.status() != WL_CONNECTED) { 
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

static const char mqtt_sensor_topic[] = "wifito868gw/sensor/status";
static const char mqtt_computherm_topic[] = "wifito868gw/computherm";
static const char mqtt_availability_topic[] = "wifito868gw/availability";
static const char mqtt_availability_message_offline[] = "offline";
static const unsigned char mqtt_availability_message_online[] = "online";

static const char mqtt_wildcard_wildcard[] = "/+";
static const char mqtt_status_subtopic[] = "/status";
static const char mqtt_control_subtopic[] = "/control";
static const char mqtt_config_subtopic[] = "/config";
static const char mqtt_pair_subtopic[] = "/pair";

void mqtthandler_setup(String mac_address) {
  mqtthandler_pubsubclient.setServer(mqtt_broker, mqtt_port);
  mqtthandler_client_id = "esp8266-wifito868gw-" + mac_address;
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
  if (mqtthandler_pubsubclient.connect(mqtthandler_client_id.c_str(), mqtt_username, mqtt_password, mqtt_availability_topic, 0, false, mqtt_availability_message_offline)) {

    // set callback
    mqtthandler_pubsubclient.setCallback(mqtthandler_callback);
  
    // subscribe
    String topicmod = mqtt_computherm_topic; topicmod += mqtt_wildcard_wildcard; topicmod += mqtt_control_subtopic;    
    mqtthandler_pubsubclient.subscribe(topicmod.c_str());
    topicmod = mqtt_computherm_topic; topicmod += mqtt_wildcard_wildcard; topicmod += mqtt_pair_subtopic;
    mqtthandler_pubsubclient.subscribe(topicmod.c_str());

    // send availability message
    mqtthandler_pubsubclient.publish(mqtt_availability_topic, mqtt_availability_message_online, sizeof(mqtt_availability_message_online)-1, true);

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
  
    char topicattrib[100];
    topicmod.toCharArray( topicattrib, 100 );
    unsigned char payloadattrb[100];
    payload.getBytes( payloadattrb, 100 );
  
    LED(true);
    Serial.printf("MQTT send topic: %s, Message: %s\n", topicattrib, payloadattrb);
    //mqtthandler_ensure_connected();
    mqtthandler_pubsubclient.publish(topicattrib, payloadattrb, payload.length(), true);
    delay(100); 
    LED(false);
  }
}

void mqtthandler_initconfig(String ttid, String payload) {
  if (mqtthandler_connected()) {
    String topicmod = mqtt_computherm_topic;
    topicmod += "/";
    topicmod += ttid;
    topicmod += mqtt_config_subtopic;

    char topicattrib[100];
    topicmod.toCharArray( topicattrib, 100 );
    unsigned char payloadattrb[100];
    payload.getBytes( payloadattrb, 100 );

    Serial.printf("MQTT init config topic: %s, Message: %s\n", topicattrib, payloadattrb);
    mqtthandler_pubsubclient.publish(topicattrib, payloadattrb, payload.length(), true);
  }
}

void mqtthandler_callback(char *topic, byte *payload, unsigned int length) {
  // split ttid from: wifito868gw/computherm/8A000/control
  String topicstr = topic;
  if (!topicstr.startsWith(mqtt_computherm_topic)) return;
  
  int delimiter_index = topicstr.indexOf("/", sizeof(mqtt_computherm_topic));
  if (delimiter_index<0) return;
  
  String ttid = topicstr.substring(sizeof(mqtt_computherm_topic), delimiter_index);
  String actionverb = topicstr.substring(delimiter_index); // include the starting "/" char as well

  // payload:
  payload[length] = '\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;
  Serial.printf("MQTT Message arrived in topic:[%s] action:[%s] message: [%s]\n", 
    topicstr.c_str(), actionverb.c_str(), message.c_str());

  // check if instance exists and if is not readonly otherwise skip
  ComputhermQThermostat *thermo = computhermqhandler_findbyid(ttid);

  if (!thermo) {
    Serial.println("MQTT unregistered thermostat - skipping control command");
  
  } else {  
    // update computherm by RF sender:
    if (actionverb == mqtt_control_subtopic) {
      if (thermo->readonly()) {
        Serial.println("MQTT read-only thermostat - skipping control command");
        
      } else {
        bool vez = (message == "ON") ? 1 : 0;
        
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
    unsigned char payloadattrb[100];
    payload.getBytes( payloadattrb, 100 );
  
    Serial.printf("MQTT send sensor update: %s, Message: %s\n", mqtt_sensor_topic, payloadattrb);
    mqtthandler_pubsubclient.publish(mqtt_sensor_topic, payloadattrb, payload.length(), false);
  }
}


void mqtthandler_loop() {
  if (mqtthandler_connected()) {
    mqtthandler_pubsubclient.loop();
    mqtthandler_lastconnected = millis();
    
  } else {
    
    long now = millis();
    
    // reconnect in a nonblocking way
    if (now - mqtthandler_lastconnectattempt > 10000) {
      if (wifihandler_connected()) {
        mqtthandler_lastconnectattempt = now;
        // attempt to reconnect every 10 sec
        if (mqtthandler_connect()) {
          mqtthandler_lastconnectattempt = 0;
          return;
        }
      }
    }

    // if no MQTT connection in like 60 sec -- turn off any manual switches
    if ((now - mqtthandler_lastconnected > mqtthandler_connection_timeout_rf_shutdown) && 
          !mqtthandler_manual_shutdown_sent) {
      Serial.println("MQTT disconnection timeout - manually shutdown all non readonly thermostats");
      mqtthandler_manual_shutdown_sent = true;
      computhermqhandler_shutdown_all_non_readonly();
    }
  }
}

////---------- rfhandler ----------//
ComputhermRF rfhandler_rf = ComputhermRF(D1, D2);
void rfhandler_setup() {
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  rfhandler_rf.startReceiver();
  Serial.println("Computherm receiver started.");
}

void rfhandler_loop() {
  if (rfhandler_rf.isDataAvailable()) {
//    last_RFMessage_millis = currentMillis; //TODO: apply and trigger on last change only
    computhermMessage msg = rfhandler_rf.getData();
    Serial.println("RF New message caught. Address: " + msg.address + " command: " + msg.command);

    ComputhermQThermostat *thermo = computhermqhandler_findbyid(msg.address);
    if (!thermo) {
      Serial.println("MQTT unregistered thermostat - still registering data over MQTT");
      mqtthandler_sendstatus(msg.address, msg.command);
      
    } else {
      thermo->set_state_update_mqtt(msg.command == "ON");
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

String ComputhermQThermostat::device_id() {
  return this->config_device_sid_;
}

bool ComputhermQThermostat::readonly() {
  return this->config_readonly_;
}

void ComputhermQThermostat::setup(String device_sid, bool readonly, int idx_channel) {
  this->config_device_sid_ = device_sid;
  this->config_readonly_ = readonly;
  this->idx_channel_ = idx_channel;

  // Revert switch to off state
  this->state_ = false;
  this->initialized_ = true;

  //TODO: reset radio?

  // initialize mqtt configchannel and state
  this->initialize_mqtt_config();
  this->set_state_update_mqtt(this->state_);
  
  // update through RF - will be auto updated on repeat for OFF, if there is no other control
  if (!this->config_readonly_) {
    // send current state (OFF) via RF
    this->set_state_update_rf(this->state_);
  }
}

void ComputhermQThermostat::set_config_device_sid(String value) {
  this->config_device_sid_ = value;
}

void ComputhermQThermostat::update() {
  if (this->initialized_) {
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
        display_blink_led();
        last_display_time_ = now;
      }
    }
  }  
}

void ComputhermQThermostat::display_blink_led() {
  if (this->state_) {
    for (int i=0; i<=idx_channel_; i++) {
      LED(true); delay(100);
      LED(false); delay(50);
    }
    delay(300);
  }
}

void ComputhermQThermostat::send_msg(uint8_t msg) {
  if (msg == MSG_NONE) return;

  Serial.printf("RF: Sending message: 0x%02x for %d:%s\n", msg, this->idx_channel_, this->config_device_sid_.c_str());

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

void ComputhermQThermostat::initialize_mqtt_config() {
  if (this->initialized_) {
    String readonly_str = this->config_readonly_ ? "true" : "false";
    String payload = "{\"readonly\": " + readonly_str + "}";
    mqtthandler_initconfig(this->config_device_sid_, payload);
  }
}

void ComputhermQThermostat::set_state_update_mqtt(bool state) {
  if (this->initialized_) {
    this->state_ = state;
    String command = state ? "ON" : "OFF";
    mqtthandler_sendstatus(this->config_device_sid_, command.c_str()); 
  }
}

void ComputhermQThermostat::set_state_update_rf(bool state) {
  if (this->initialized_) {
    this->state_ = state;
    this->pending_msg_ = state ? MSG_HEAT_ON : MSG_HEAT_OFF;

    // if thermostat is ON, blink LED to show state X times
    this->display_blink_led();
  }
}

void ComputhermQThermostat::on_pairing() {
  if (this->initialized_) {
    // enqueue pairing
    this->pending_msg_ = MSG_PAIR;
  }
}

////---------- computhermqhandler ----------//
static const unsigned int thermo_count = SECRET_THERMO_COUNT;
static const char thermo_id[thermo_count][6] = SECRET_THERMO_IDS;
static const bool thermo_readonly[thermo_count] = SECRET_THERMO_READONLY;
ComputhermQThermostat thermos[4];

void computhermqhandler_setup() {
  for (int i = 0; i < thermo_count; i++) {
    thermos[i].setup(thermo_id[i], thermo_readonly[i], i);
  }
}

void computhermqhandler_loop() {
  for (int i = 0; i < thermo_count; i++) {
    thermos[i].update();
  }
}

ComputhermQThermostat* computhermqhandler_findbyid(String device_sid) {
  for (int i = 0; i < thermo_count; i++) {
    if (thermos[i].device_id() == device_sid) {
      return &thermos[i];
    }
  }
  return NULL;
}

void computhermqhandler_shutdown_all_non_readonly() {
  for (int i = 0; i < thermo_count; i++) {
    if (!thermos[i].readonly()) {
      thermos[i].set_state_update_rf(false);
    }
  }
}
