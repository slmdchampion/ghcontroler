#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <OneWire.h>
#include <ArduinoJson.h>
#include "Secrets.h"

//  I2C device address is 0 1 0 0   A2 A1 A0
#define EXP0_ADDRESS (0x4 << 3 | 0x0)
#define EXP_INPUT0 0x12
#define EXP_INPUT1 0x13
#define EXP_OUTPUT0 0x12
#define EXP_OUTPUT1 0x13
#define EXP_CONFIG0  0
#define EXP_CONFIG1 1

// thing
#define RtcSquareWavePin 4

// nodemcu pin D7
// #define RtcSquareWavePin 13



// storage for Json parsing
StaticJsonDocument<512> doc;

// MQTT: topics
const char* MQTT_RELAY_STATE_TOPIC = "ghcontroller/status";
const char* MQTT_RELAY_COMMAND_TOPIC = "ghcontroller/valve";
const char* MQTT_TIMER_COMMAND_TOPIC = "ghcontroller/timer";
const char* MQTT_TIMER_TIME_TOPIC = "ghcontroller/timer/state";
const char* MQTT_TIMER_TIMER_STATUS_TOPIC = "ghcontroller/timer/status";
const char* TIMER_SET_KEY = "set time";
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

//array of current relay states
boolean m_relay_state[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
// array of whether a relay is manually programed
boolean m_relay_manual[16] = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};

WiFiClient wifiClient;
PubSubClient client(wifiClient);
RtcDS3231<TwoWire> Rtc(Wire);
RtcDateTime current_time;
uint8_t curent_weekday;
uint8_t minute_count;

volatile bool interuptFlag = false;

// Timer programing vars

uint16_t relay_duration[5][16] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

uint32_t program_start_time[5];

// Sunday, Monday, Tuesday, Wednesday, Thursday, Friday, Saturday
bool program_days[5][7] = {{false, false, false, false, false, false, false}, {false, false, false, false, false, false, false}, {false, false, false, false, false, false, false},
                            {false, false, false, false, false, false, false}, {false, false, false, false, false, false, false}};

// RTC interupt
void ICACHE_RAM_ATTR InteruptServiceRoutine(){
  interuptFlag = true;
  // Serial.println("ISR");
}

//function called to set gpio state
void Write_Gpio(uint8_t relay_no, int state) {
    uint8_t port0_byte = 0, port1_byte = 0;
    Serial.println("relay state");
    Serial.println("0123456701234567");
  
       // store gpio state

    if (state == 1) {
        m_relay_state[relay_no] = true;
    } else {
        m_relay_state[relay_no] = false;
    }
    // set or clear bits in the port0_byte and port1_byte to output to i2c
    for (uint8_t i = 0; i < 8; i++) {
        Serial.print(m_relay_state[i]);
        if (m_relay_state[i]) {
           bitClear(port0_byte,i);
        } else {
            bitSet(port0_byte,i);
        }
    }
    for (uint8_t i = 8; i < 16; i++) {
        Serial.print(m_relay_state[i]);
        if (m_relay_state[i]) {
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
    Serial.print("port 0 byte: ");
    Serial.println(port0_byte, BIN);
    Wire.beginTransmission(EXP0_ADDRESS);
    Wire.write(EXP_OUTPUT1);
    Wire.write(port1_byte);
    Wire.endTransmission();
    
    Serial.print("port 1 byte: ");
    Serial.println(port1_byte, BIN);
    Wire.endTransmission();

}
// function called to publish the state of the light (on/off)
void publishRelayState(uint8_t relay_no) {
  String topic = String(MQTT_RELAY_STATE_TOPIC);
  char buffer[60];

  topic += String(relay_no +1); 
  topic.toCharArray(buffer, 60);

  if (m_relay_state[relay_no]) {
    client.publish(buffer, RELAY_ON, true);
    Serial.print(buffer);
    Serial.println("publish on...");
  } else {
    client.publish(buffer, RELAY_OFF, true);
    Serial.print(buffer);
    Serial.println("publish off...");
  }
}

// function called to turn on/off the light
void setRelayState(uint8_t relay_no) {
  if (m_relay_state[relay_no]) {
    Write_Gpio(relay_no, 1);
    Serial.print("INFO: Turn relay");
    Serial.print(relay_no);
    Serial.println(" on...");
  } else {
    Write_Gpio(relay_no, 0);
    Serial.print("INFO: Turn relay");
    Serial.print(relay_no);
    Serial.println(" off...");
  }
}


// function called when a MQTT message arrived
//  TODO  add mqtt handler for changing programmed timers
void callback(char* p_topic, byte* p_payload, unsigned int p_length)
{
  Serial.println(String(p_topic));
  // concat the payload into a string
  String payload,key;
  
  for (uint8_t i = 0; i < p_length; i++)
  {
    payload.concat((char)p_payload[i]);
  }
  Serial.println(payload);
  // handle message topic
  if (String(MQTT_TIMER_COMMAND_TOPIC).equals(p_topic)) {
    // Serial.println("timer command");
     // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, payload);

  // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    if (doc.containsKey(TIMER_SET_KEY)){
      uint32_t inttime = doc[TIMER_SET_KEY];
      Rtc.SetDateTime(RtcDateTime(inttime));
      // TODO: send settime reply mqtt
    }
    for (uint8_t p=0; p<5; p++) {
      key = String("program ") + String(p+1) + String(" start");  
      if (doc.containsKey(key.c_str())) {
        RtcDateTime now = Rtc.GetDateTime();
        // date/time up to midnight
        RtcDateTime start_time = RtcDateTime(now.Year(),now.Month(),now.Day(),0,0,0);

        int start = doc[key.c_str()];
        program_start_time[p] = start_time.TotalSeconds() + start;
        Serial.print("program ");
        Serial.print(p+1);
        Serial.print(" start time set to ");
        Serial.println(program_start_time[p]);
      }
      key = String("program ") + String(p+1) + String(" days"); 
      if (doc.containsKey(key.c_str())){
        // Serial.println("Program 1 Days: ");
        for (uint8_t i=0; i<7; i++)
        {
          program_days[p][i] = doc[key.c_str()][i];
          Serial.println(program_days[p][i]);
        }
      }
      key = String("program ") + String(p+1) + String(" times"); 
      if (doc.containsKey(key.c_str()))
      {
        Serial.print("Program ");
        Serial.print(p+1);
        Serial.println(" times");
        for (uint8_t i=0; i<16; i++)
        {
          relay_duration[p][i] = doc[key.c_str()][i];
          // Serial.println(relay_duration[p][i]);
        }
      }
    }
  }
  for (uint8_t i=0; i < 16; i++)
  {
      
    if (String(MQTT_RELAY_COMMAND_TOPIC + String(i + 1)).equals(p_topic))
    {
        // test if the payload is equal to "ON" or "OFF"
        if (payload.equals(String(RELAY_ON)))
        {
          m_relay_manual[i] = !m_relay_manual[i];
          if (m_relay_state[i] != true)
          {
            m_relay_state[i] = true;
            setRelayState(i);
            publishRelayState(i);
          }
        } else if (payload.equals(String(RELAY_OFF)))
        {
          m_relay_manual[i] = !m_relay_manual[i];
          if (m_relay_state[i] != false)
          {
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
  while (!client.connected()) {
    Serial.println("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
      // Once connected, publish an announcement...
      for (uint8_t i=0; i < 16; i++) {
        publishRelayState(i);
      }
      // ... and resubscribe
      topic = String(MQTT_TIMER_COMMAND_TOPIC);
      topic.toCharArray(buffer,60);
      Serial.println(buffer);
      client.subscribe(buffer);

      for (uint8_t i=0; i < 16; i++) {
        topic = String(MQTT_RELAY_COMMAND_TOPIC);
        topic += String(i + 1);
        topic.toCharArray(buffer, 60);
        Serial.println(buffer);
        client.subscribe(buffer);
      }
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
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
  uint32_t program_time_total; //program time in seconds
  String payload, topic;
  bool running[5] = {false, false, false, false, false};

  if (interuptFlag){
    minute_count++;
    if (minute_count == 2){
      // Send total seconds to the time topic
      payload = "{\"time\" : ";
      current_time = Rtc.GetDateTime();
      payload += String(current_time.TotalSeconds());
      payload += "}";
      client.publish(MQTT_TIMER_TIME_TOPIC, payload.c_str());
      Serial.println(payload);
      Serial.print(current_time.Hour());
      Serial.print(":");
      Serial.print(current_time.Minute());
      Serial.print(":");
      Serial.println(current_time.Second());
      minute_count = 0;
    }
    Rtc.LatchAlarmsTriggeredFlags();
    interuptFlag = false;  //interupt has been handled
    // code for timers
    Serial.println("Timer Parsing: ");
    current_time = Rtc.GetDateTime();
    curent_weekday = current_time.DayOfWeek();

    for (uint8_t i = 0; i < 5; i++){
      if (program_days[i][curent_weekday]){
        // does this program run today
        Serial.print(i);
        Serial.println(" Runs today");
        program_time_total = program_start_time[i];
        for (uint8_t j=0; j<16; j++)
        {
          if (relay_duration[i][j] != 0){
            Serial.print(j);
            Serial.println(" has time");
            // is there time for this relay
            Serial.print("current time: ");
            Serial.println(current_time.TotalSeconds());
            Serial.print("Program time: ");
            Serial.println(program_time_total);
            if ((current_time.TotalSeconds() >= program_time_total) && (current_time.TotalSeconds() < (program_time_total + relay_duration[i][j]*60))){
              running[i] = true;
              Serial.print(j);
              Serial.println(" should run");
              Serial.println(m_relay_state[j]);
              Serial.println(m_relay_manual[j]);
              // is current time within the run time of this relay so it needs to be in the on state
              if (!m_relay_state[j] && !m_relay_manual[j]){
                Serial.print(j);
                Serial.println(" turning on");
                // is the relay off but not off manually
                m_relay_state[j] = true;
                setRelayState(j);
                publishRelayState(j);
              }                
            }
            else if (m_relay_state[j] && !m_relay_manual[j]){              
              Serial.print(j);
              Serial.println(" turning off");
              // else it's supposed to be off but is it on and not o
              m_relay_state[j] = false;
              setRelayState(j);
              publishRelayState(j);
            }
            program_time_total += relay_duration[i][j]*60; //add this relay's time to the total program time
          }
        }
      }
    }
    // puplish program running status
    for (uint8_t i=0; i<5; i++){
      topic = String(MQTT_TIMER_TIMER_STATUS_TOPIC) + String(i+1);
      if (running[i]){
        client.publish(topic.c_str(), "true");
        Serial.print("Program ");
        Serial.print(i+1);
        Serial.println(" is running");
      }
      else {
        client.publish(topic.c_str(), "false");
        Serial.print("Program ");
        Serial.print(i+1);
        Serial.println(" is not running");
      }
    }
  }
  if (!client.connected()) {
    reconnect();
  }
  delay(500);
  client.loop();
}