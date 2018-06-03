#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

//  I2C device address is 0 1 0 0   A2 A1 A0
#define EXP0_ADDRESS (0x4 << 3 | 0x0)
#define EXP_INPUT0 0
#define EXP_INPUT1 1
#define EXP_OUTPUT0 2
#define EXP_OUTPUT1 3
#define EXP_CONFIG0 6
#define EXP_CONFIG1 7

// Wifi: SSID and password
const char* WIFI_SSID = "robertjoseph";
const char* WIFI_PASSWORD = "allmylove";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "office_light1";
const PROGMEM char* MQTT_SERVER_IP = "10.0.0.30";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "homeassistant";
const PROGMEM char* MQTT_PASSWORD = "home23";

// MQTT: topics
const char* MQTT_RELAY_STATE_TOPIC = "gardenplot/greenhouse/controller/status";
const char* MQTT_RELAY_COMMAND_TOPIC = "gardenplot/greenhouse/controller/valve";

// payloads by default (on/off)
const char* RELAY_ON = "ON";
const char* RELAY_OFF = "OFF";

boolean m_relay_state[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

WiFiClient wifiClient;
PubSubClient client(wifiClient);

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
           bitSet(port0_byte,i);
        } else {
            bitClear(port0_byte,i);
        }
    }
    for (uint8_t i = 8; i < 16; i++) {
        Serial.print(m_relay_state[i]);
        if (m_relay_state[i]) {
            bitSet(port1_byte,i-8);
        } else {
            bitClear(port1_byte,i-8);
        }
    }
    Serial.println();
    Wire.beginTransmission(EXP0_ADDRESS);
    Wire.write(EXP_OUTPUT0);
    Wire.write(port0_byte);
    Serial.print("port 0 byte: ");
    Serial.println(port0_byte, BIN);
    Wire.write(port1_byte);
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
  } else {
    client.publish(buffer, RELAY_OFF, true);
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
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
  Serial.println(payload);
  // handle message topic
  for (uint8_t i=0; i < 16; i++) {
      
    if (String(MQTT_RELAY_COMMAND_TOPIC + String(i)).equals(p_topic)) {
        // test if the payload is equal to "ON" or "OFF"
        if (payload.equals(String(RELAY_ON))) {
            if (m_relay_state[i] != true) {
                m_relay_state[i] = true;
                setRelayState(i);
                publishRelayState(i);
            }
        } else if (payload.equals(String(RELAY_OFF))) {
            if (m_relay_state[i] != false) {
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
      for (uint8_t i=0; i < 16; i++) {
        topic = String(MQTT_RELAY_COMMAND_TOPIC);
        topic += String(i);
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
  Wire.begin(D1,D2);
  Wire.beginTransmission(EXP0_ADDRESS);
  Wire.write(EXP_CONFIG0);
  // send 1 for output 0 for input to the config registers
  Wire.write(0x00);  //set port 0 to output
  Wire.write(0x00);  //set port 1 to output
  Wire.endTransmission();
  // set all bits to off
  Wire.beginTransmission(EXP0_ADDRESS);
  Wire.write(EXP_OUTPUT1);
  Wire.write(0b00000000);
  Wire.write(0b00000000);
  Wire.endTransmission();

  // initialize serial debug and monitor
  Serial.begin(9600);

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

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);
}

void loop() {
    // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}