#include <Arduino.h>

#include <PubSubClient.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <OneWire.h>
#include <ArduinoJson.h>
#include "Secrets.h"

// TODO:   verify -->program start needs to be from midnight not absolute, fix relay on/off logic 
// things to add: change any global variables, make sure can see all globals
//  I2C device address is 0 1 0 0   A2 A1 A0
#define EXP0_ADDRESS (0x4 << 3 | 0x0)
#define EXP_INPUT0 0x12
#define EXP_INPUT1 0x13
#define EXP_OUTPUT0 0x12
#define EXP_OUTPUT1 0x13
#define EXP_CONFIG0 0
#define EXP_CONFIG1 1

#ifdef BOARDESP32
  #include "Wifi.h"
  #define RtcSquareWavePin 4
    // MQTT: topics
  const char* MQTT_RELAY_STATE_TOPIC = "ghcontroller/status";
  const char* MQTT_RELAY_CONNECTION_TOPIC = "ghcontroller/connection";
  const char* MQTT_RELAY_COMMAND_TOPIC = "ghcontroller/valve";
  const char* MQTT_TIMER_COMMAND_TOPIC = "ghcontroller/timer";
  const char* MQTT_TIMER_TIME_TOPIC = "ghcontroller/timer/state";
  const char* MQTT_TIMER_TIMER_STATUS_TOPIC = "ghcontroller/timer/status";
  const char* TIMER_SET_KEY = "set time";
  const char* OVERRIDE_SET_KEY = "override ";
  // Wifi: SSID and password
  const char* WIFI_SSID = MY_WIFI_SSID;
  const char* WIFI_PASSWORD = MY_WIFI_PASSWORD;

  // MQTT: ID, server IP, port, username and password
  const PROGMEM char* MQTT_CLIENT_ID = "ghcontroller";
  const PROGMEM char* MQTT_SERVER_IP = MY_MQTT_IP;
  const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
  const PROGMEM char* MQTT_USER = MY_MQTT_USER;
  const PROGMEM char* MQTT_PASSWORD = MY_MQTT_PASSWORD;

  // payloads by default (on/off)
  const char* RELAY_ON = "ON";
  const char* RELAY_OFF = "OFF";


// // thing
#elif BOARDTHING
  #include <ESP8266WiFi.h>
  #define RtcSquareWavePin 4
    // MQTT: topics
  const char* MQTT_RELAY_STATE_TOPIC = "ghcontroller/status";
  const char* MQTT_RELAY_CONNECTION_TOPIC = "ghcontroller/connection";
  const char* MQTT_RELAY_COMMAND_TOPIC = "ghcontroller/valve";
  const char* MQTT_TIMER_COMMAND_TOPIC = "ghcontroller/timer";
  const char* MQTT_TIMER_TIME_TOPIC = "ghcontroller/timer/state";
  const char* MQTT_TIMER_TIMER_STATUS_TOPIC = "ghcontroller/timer/status";
  const char* TIMER_SET_KEY = "set time";
  const char* OVERRIDE_SET_KEY = "override ";
  // Wifi: SSID and password
  const char* WIFI_SSID = MY_WIFI_SSID;
  const char* WIFI_PASSWORD = MY_WIFI_PASSWORD;

  // MQTT: ID, server IP, port, username and password
  const PROGMEM char* MQTT_CLIENT_ID = "ghcontroller";
  const PROGMEM char* MQTT_SERVER_IP = MY_MQTT_IP;
  const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
  const PROGMEM char* MQTT_USER = MY_MQTT_USER;
  const PROGMEM char* MQTT_PASSWORD = MY_MQTT_PASSWORD;

  // payloads by default (on/off)
  const char* RELAY_ON = "ON";
  const char* RELAY_OFF = "OFF";

#elif defined(BOARDNODEMCU)
  #include <ESP8266WiFi.h>
  #define RtcSquareWavePin 13
  // MQTT: topics
  const char* MQTT_RELAY_STATE_TOPIC = "ghcontroller2/status";
  const char* MQTT_RELAY_COMMAND_TOPIC = "ghcontroller2/valve";
  const char* MQTT_TIMER_COMMAND_TOPIC = "ghcontroller2/timer";
  const char* MQTT_TIMER_TIME_TOPIC = "ghcontroller/timer2/state";
  const char* MQTT_TIMER_TIMER_STATUS_TOPIC = "ghcontroller2/timer/status";
  const char* TIMER_SET_KEY = "set time";
  const char* OVERRIDE_SET_KEY = "override ";
  // Wifi: SSID and password
  const char* WIFI_SSID = MY_WIFI_SSID;
  const char* WIFI_PASSWORD = MY_WIFI_PASSWORD;

  // MQTT: ID, server IP, port, username and password
  const PROGMEM char* MQTT_CLIENT_ID = "ghcontroller2";
  const PROGMEM char* MQTT_SERVER_IP = MY_MQTT_IP;
  const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
  const PROGMEM char* MQTT_USER = MY_MQTT_USER;
  const PROGMEM char* MQTT_PASSWORD = MY_MQTT_PASSWORD;

  // payloads by default (on/off)
  const char* RELAY_ON = "ON";
  const char* RELAY_OFF = "OFF";

#else
  #define RtcSquareWavePin 7
// nodemcu pin D7
#endif

// storage for Json parsing
StaticJsonDocument<512> doc;


//array of current relay states
boolean m_relay_state[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
// array of whether a relay is manually programed
boolean m_relay_manual[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

WiFiClient wifiClient;
WiFiClient telnet_client;
PubSubClient client(wifiClient);
int port = 23;  //Port number
WiFiServer telnet_server(port);
bool connected_flag = false;
RtcDS3231<TwoWire> Rtc(Wire);
RtcDateTime current_time;
uint8_t curent_weekday;
uint8_t minute_count;

volatile bool interuptFlag = false;

// Timer programing vars

uint16_t relay_duration[5][16] = {{60, 0, 0, 0, 60, 0, 0, 0, 45, 0, 0, 0, 0, 0, 0, 0}, {0, 60, 0, 0, 0, 70, 0, 0, 0, 45, 0, 0, 0, 0, 0, 0}, {0, 0, 60, 0, 0, 0, 45, 0, 0, 0, 45, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 60, 0, 0, 0, 45, 0, 0, 0, 45, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

uint32_t program_start_time[5] = {21600, 21600, 21600, 21600, 0};

// Sunday, Monday, Tuesday, Wednesday, Thursday, Friday, Saturday
bool program_days[5][7] = {{true, true, true, true, true, true, true}, {true, true, true, true, true, true, true}, {true, true, true, true, true, true, true},
                            {true, true, true, true, true, true, true}, {false, false, false, false, false, false, false}};

// RTC interupt
void ICACHE_RAM_ATTR InteruptServiceRoutine(){
  interuptFlag = true;
  // Serial.println("ISR");
}

// output debug info
void DebugOutput(String output_string){
  Serial.println(output_string);
  telnet_client.println(output_string);
}
// set i2c gpio state
void Write_Gpio(uint8_t relay_no, int state) {
    uint8_t port0_byte = 0, port1_byte = 0;
    DebugOutput("relay state");
    DebugOutput("0123456701234567");
    // store gpio state
    if (state == 1){
        m_relay_state[relay_no] = true;
    } else {
        m_relay_state[relay_no] = false;
    }
    // set or clear bits in the port0_byte and port1_byte to output to i2c
    for (uint8_t i = 0; i < 8; i++){
        Serial.print(m_relay_state[i]);
        if (m_relay_state[i]) {
           bitClear(port0_byte,i);
        } else {
            bitSet(port0_byte,i);
        }
    }
    for (uint8_t i = 8; i < 16; i++){
        Serial.print(m_relay_state[i]);
        if (m_relay_state[i]){
            bitClear(port1_byte,i-8);
        } else {
            bitSet(port1_byte,i-8);
        }
    }
    Serial.println();
    Wire.beginTransmission(EXP0_ADDRESS);
    Wire.write(EXP_OUTPUT0);
    Wire.write(port0_byte);
    Wire.endTransmission();
    DebugOutput("port 0 byte: " + String(port0_byte, BIN));
    Wire.beginTransmission(EXP0_ADDRESS);
    Wire.write(EXP_OUTPUT1);
    Wire.write(port1_byte);
    Wire.endTransmission();    
    DebugOutput("port 1 byte: " + String(port1_byte, BIN));
    Wire.endTransmission();

}
// function called to publish the state of the relay (on/off)
void publishRelayState(uint8_t relay_no) {
  String topic = String(MQTT_RELAY_STATE_TOPIC);
  char buffer[60];

  topic += String(relay_no +1); 
  topic.toCharArray(buffer, 60);
  if (m_relay_state[relay_no]) {
    client.publish(buffer, RELAY_ON, true);
    DebugOutput(buffer + String("publish on..."));
  } else {
    client.publish(buffer, RELAY_OFF, true);
    DebugOutput(buffer + String("publish off..."));
  }
}

// function called to turn on/off the relay
void setRelayState(uint8_t relay_no) {
  if (m_relay_state[relay_no]) {
    Write_Gpio(relay_no, 1);
    DebugOutput("INFO: Turn relay" + String(relay_no) + " on...");
  } else {
    Write_Gpio(relay_no, 0);
    DebugOutput("INFO: Turn relay" + String(relay_no) + " off...");
  }
}

// function called when a MQTT message arrived
//  TODO  add mqtt handler for changing programmed timers
void callback(char* p_topic, byte* p_payload, unsigned int p_length){
  String payload,key;
  
  DebugOutput("topic: " + String(p_topic));
  // concat the payload into a string
  for (uint8_t i = 0; i < p_length; i++){
    payload.concat((char)p_payload[i]);
  }
  DebugOutput("payload: " + payload);
  // handle message topic
  if (String(MQTT_TIMER_COMMAND_TOPIC).equals(p_topic)) {
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, payload);
  // Test if parsing succeeds.
    if (error) {
      DebugOutput(String(F("deserializeJson() failed: ")) + String(error.c_str()));
      return;
    }
    if (doc.containsKey(TIMER_SET_KEY)){
      uint32_t inttime = doc[TIMER_SET_KEY];
      Rtc.SetDateTime(RtcDateTime(inttime));
    }
    // process "override x" json ---> {"override 1 : true"} ---> relay 1 is set soley by home assistant don't use internal timers for it
    for (uint8_t i=0; i<16; i++){
      key = OVERRIDE_SET_KEY + String(i+1);
      if (doc.containsKey(key.c_str())){
        m_relay_manual[i] = doc[key.c_str()];
        DebugOutput("Relay " + String(i+1) + " override set to " + String(m_relay_manual[i]));        
      }
    }
    //  process "program x start" json ---> {"program 1 start" : 28860} ---> set program 1 start to 8:01 (28860 seconds from midnight) 8*3600 + 1*60
    for (uint8_t p=0; p<5; p++) {
      key = String("program ") + String(p+1) + String(" start");  
      if (doc.containsKey(key.c_str())) {
        int start = doc[key.c_str()];
        program_start_time[p] =  start;
        m_relay_manual[p] = false;
        DebugOutput("program " + String(p+1) + " start time set to " + String(program_start_time[p]));
      }
      key = String("program ") + String(p+1) + String(" days"); 
      if (doc.containsKey(key.c_str())){
        for (uint8_t i=0; i<7; i++){
          program_days[p][i] = doc[key.c_str()][i];
          DebugOutput(String(program_days[p][i]));
          yield();
        }
      }
      key = String("program ") + String(p+1) + String(" times"); 
      if (doc.containsKey(key.c_str())){
        DebugOutput("Program " + String(p+1) + " times");
        for (uint8_t i=0; i<16; i++){
          relay_duration[p][i] = doc[key.c_str()][i];
          DebugOutput(String(relay_duration[p][i]));
          yield();
        }
      }
    }
  }
  for (uint8_t i=0; i < 16; i++){      
    if (String(MQTT_RELAY_COMMAND_TOPIC + String(i + 1)).equals(p_topic)){
        // test if the payload is equal to "ON" or "OFF"
        if (payload.equals(String(RELAY_ON))){
          if (m_relay_state[i] != true){
            m_relay_state[i] = true;
            setRelayState(i);
            publishRelayState(i);
          }
        } else if (payload.equals(String(RELAY_OFF))){
          if (m_relay_state[i] != false){
            m_relay_state[i] = false;
            setRelayState(i);
            publishRelayState(i);
          }
        }
    }
  }
}

void reconnect() {
  char buffer[60];
  String topic;

  // Loop until we're reconnected
  while (!client.connected()){
    Serial.println("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_RELAY_CONNECTION_TOPIC, 0, true, "Offline")) {
      Serial.println("INFO: connected");
      // Once connected, publish an announcement...
      client.publish(MQTT_RELAY_CONNECTION_TOPIC,"Online", true);
      for (uint8_t i=0; i < 16; i++){
        publishRelayState(i);
      }
      // ... and resubscribe
      topic = String(MQTT_TIMER_COMMAND_TOPIC);
      topic.toCharArray(buffer,60);
      DebugOutput(buffer);
      client.subscribe(buffer);
      for (uint8_t i=0; i < 16; i++){
        topic = String(MQTT_RELAY_COMMAND_TOPIC);
        topic += String(i + 1);
        topic.toCharArray(buffer, 60);
        DebugOutput(buffer);
        client.subscribe(buffer);
      }
    } else {
      DebugOutput("ERROR: failed, rc=" + String(client.state()) + "DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  minute_count = 0;
    // initialize serial debug and monitor
  Serial.begin(9600);
     // set the interupt pin to input mode
  pinMode(RtcSquareWavePin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(RtcSquareWavePin), InteruptServiceRoutine, FALLING);
  Serial.println(RtcSquareWavePin);
//  Wire.begin();
  Rtc.Begin();

  Wire.beginTransmission(EXP0_ADDRESS);
  Wire.write(EXP_CONFIG0);  // send 1 for output 0 for input to the config registers
  Wire.write(0x00);  //set port 0 to output
  Wire.endTransmission();

  Wire.beginTransmission(EXP0_ADDRESS);
  Wire.write(EXP_CONFIG1);
  Wire.write(0x00);  //set port 1 to output
  Wire.endTransmission();

  // set all bits to off
  Wire.beginTransmission(EXP0_ADDRESS);
  Wire.write(EXP_OUTPUT0);
  Wire.write(0b11111111);
  Wire.endTransmission();
  Wire.beginTransmission(EXP0_ADDRESS);
  Wire.write(EXP_OUTPUT1);
  Wire.write(0b11111111);
  Wire.endTransmission();
  
  // init the WiFi connection
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  WiFi.mode(WIFI_STA);
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("INFO: WiFi connected");
  Serial.print("INFO: IP address: ");
  Serial.println(WiFi.localIP());
  telnet_server.begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if (!Rtc.IsDateTimeValid()) 
   {
       if (Rtc.LastError() != 0)
       {
           // we have a communications error
           // see https://www.arduino.cc/en/Reference/WireEndTransmission for 
           // what the number means
           Serial.print("RTC communications error = ");
           Serial.println(Rtc.LastError());
       }
       else
       {
           Serial.println("RTC lost confidence in the DateTime!");
           Rtc.SetDateTime(compiled);
       }
   }

   if (!Rtc.GetIsRunning())
   {
       Serial.println("RTC was not actively running, starting now");
       Rtc.SetIsRunning(true);
   }

   RtcDateTime now = Rtc.GetDateTime();
   if (now < compiled) 
   {
       Serial.println("RTC is older than compile time!  (Updating DateTime)");
       Rtc.SetDateTime(compiled);
   }
   Serial.print("Current time up to midnight: ");
   Serial.println(Rtc.GetDateTime().TotalSeconds()); 
   client.publish(MQTT_TIMER_TIMER_STATUS_TOPIC,String(Rtc.GetDateTime().TotalSeconds()).c_str());
   Rtc.Enable32kHzPin(false);


   Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth); 
    RtcDateTime alarmTime = now + 88; // into the future
    DS3231AlarmOne alarm1(
            alarmTime.Day(),
            alarmTime.Hour(),
            alarmTime.Minute(), 
            alarmTime.Second(),
            DS3231AlarmOneControl_HoursMinutesSecondsMatch);
    Rtc.SetAlarmOne(alarm1);

    // Alarm 2 set to trigger at the top of the minute
    DS3231AlarmTwo alarm2(0, 0, 0, DS3231AlarmTwoControl_OncePerMinute);
    Rtc.SetAlarmTwo(alarm2);

    // throw away any old alarm state before we ran
    Rtc.LatchAlarmsTriggeredFlags();

    // setup external interupt 
    attachInterrupt(digitalPinToInterrupt(RtcSquareWavePin), InteruptServiceRoutine, FALLING);

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);
}

void loop() {
  uint32_t program_time_total, midnight; //program time in seconds
  String payload, topic;
  bool running[5] = {false, false, false, false, false};
  bool relay_turn_on[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
  WiFiClient throwaway_client = telnet_server.available();

  if (throwaway_client) {
    connected_flag = true;
    telnet_client = throwaway_client;
    telnet_client.flush();
    DebugOutput("Client Connected");
    
    }

  if (interuptFlag){
    minute_count++;
    if (minute_count == 2){
      payload = "{\"time\" : ";
      current_time = Rtc.GetDateTime();
      payload += String(current_time.TotalSeconds());
      payload += "}";
      client.publish(MQTT_TIMER_TIME_TOPIC, payload.c_str());
      DebugOutput(payload + String(current_time.Hour()) + ":" + String(current_time.Minute()) + ":" + String(current_time.Second()));
      DebugOutput(String(millis()) + "  millis");
      minute_count = 0;
      for (uint8_t i=0; i<5; i++){
        payload = "{\"program " + String(i+1) + " times\": [";
        for (uint8_t j=0; j<16; j++){
          payload += String(relay_duration[i][j]);
          if (j<15){
            payload += ", ";
          }
        }
        payload += "]}";
        DebugOutput(payload);
      }
      for (uint8_t i=0; i<5; i++){
        payload = "{\"program " + String(i+1) + " days\": [";
        for (uint8_t j=0; j<7; j++){
          payload += String(program_days[i][j]);
          if (j<6){
            payload += ", ";
          }
        }
        payload += "]}";
        DebugOutput(payload);
      }
      for (uint8_t j=0; j<5; j++){
        payload = "{\"program " + String(j+1) + " start\": " + String(program_start_time[j]) + "}";
        DebugOutput(payload);
      }
      for (uint8_t j=0; j<16; j++){
        payload = "{\"relay " + String(j+1) + " state\": " + String(m_relay_state[j]) + "}";
        DebugOutput(payload);
      }
      for (uint8_t j=0; j<16; j++){
        payload = "{\"relay " + String(j+1) + " override\": " + String(m_relay_manual[j]) + "}";
        DebugOutput(payload);
      }
    }
    Rtc.LatchAlarmsTriggeredFlags();
    interuptFlag = false;  //interupt has been handled
    // code for timers
    DebugOutput("Timer Parsing: ");
    current_time = Rtc.GetDateTime();
    curent_weekday = current_time.DayOfWeek();
    midnight = RtcDateTime(current_time.Year(),current_time.Month(),current_time.Day(),0,0,0).TotalSeconds();

    for (uint8_t i = 0; i < 5; i++){
      if (program_days[i][curent_weekday]){
        // does this program run today
        DebugOutput(String(i) + " Runs today");
        program_time_total = program_start_time[i] + midnight;
        for (uint8_t j=0; j<16; j++)
        {
          if (relay_duration[i][j] != 0){
            DebugOutput(String(j) + " has time");
            // is there time for this relay
            DebugOutput("current time: " + String(current_time.TotalSeconds()));
            DebugOutput("Program time: " + String(program_time_total));
            if ((current_time.TotalSeconds() >= program_time_total) && (current_time.TotalSeconds() < (program_time_total + relay_duration[i][j]*60))){
              running[i] = true;
              DebugOutput(String(j) + " should run");
              DebugOutput(String(m_relay_state[j]));
              DebugOutput(String(m_relay_manual[j]));
              // is current time within the run time of this relay so it needs to be in the on state
              if (!m_relay_manual[j]){
   
                // is the relay set manually
                relay_turn_on[j] = true;
                //m_relay_state[j] = true;
                //setRelayState(j);
                //publishRelayState(j);
              }                
            }
            //else if (m_relay_state[j] && !m_relay_manual[j]){              
              //DebugOutput(String(j) + " turning off");
              // else it's supposed to be off but is it on and not o
              //m_relay_state[j] = false;
              //setRelayState(j);
              //publishRelayState(j);
            //}
            program_time_total += relay_duration[i][j]*60; //add this relay's time to the total program time
          }
        yield();
        }
      }
    }
    for (uint8_t j=0; j<16; j++){
      if (!m_relay_manual[j] && (m_relay_state[j] != relay_turn_on[j])){
        if (relay_turn_on[j]){
          DebugOutput(String(j) + " turning on");
        }
        else {
          DebugOutput(String(j) + " turning off");
        }
        m_relay_state[j] = relay_turn_on[j];
        setRelayState(j);
        publishRelayState(j);
      }
      yield();
    }

    // puplish program running status
    for (uint8_t i=0; i<5; i++){
      topic = String(MQTT_TIMER_TIMER_STATUS_TOPIC) + String(i+1);
      if (running[i]){
        client.publish(topic.c_str(), "true");
        DebugOutput("Program " + String(i+1) + " is running");
      }
      else {
        client.publish(topic.c_str(), "false");
        DebugOutput("Program " + String(i+1) + " is not running");
      }
      yield();
    }
  }
  if (!client.connected()) {
    reconnect();
  }
  yield();
  client.loop();
}