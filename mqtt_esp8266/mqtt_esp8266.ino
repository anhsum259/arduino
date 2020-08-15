#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h> //include Wire.h library
#include <ArduinoJson.h>
SimpleKalmanFilter KalmanFilter(1, 1, 0.01);




int analogInput = 0;
float vout = 0.0;
float vin = 0.0;
float vin1 = 0.0;
float R1 = 100000.0; // resistance of R1 (100K) -see text!
float R2 = 10000.0; // resistance of R2 (10K) - see text!
int value = 0;



unsigned long time1 = 0;
unsigned long time2 = 0;

const char* ssid = "ZTE-d6a7e1"; // Enter your WiFi name
const char* password =  "78312bd6"; // Enter WiFi password
const char* mqttServer = "192.168.1.17";
const int mqttPort = 1883;
const char* mqttUser = "sunnyhome";
const char* mqttPassword = "0985328757";

byte willQoS = 1;                          //qos of publish (see README)

/* khai bao dc nhan I2C */
int slaveAddress = 0x7E;


WiFiClient espClient;
PubSubClient client(espClient);
int HeatingPin = 14;//GPIO05
String switch1;
String strTopic;
String strPayload;


void setup() {

  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP8266-Solar", mqttUser, mqttPassword )) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }
  //client.publish("home/solar", "ESP8266-solarclean-ready"); //Topic name
  pinMode(HeatingPin, OUTPUT);
  //client.subscribe("home/solar/robot/esp8266/14");
  /* scan I2C device */
  Wire.begin();
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  strTopic = String((char*)topic);
  if (strTopic == "home/solar/robot/esp8266/14")
  {
    switch1 = String((char*)payload);
    //Serial.println(switch1);
    if (switch1 == "1")
    {
      Serial.println("ON");
      digitalWrite(HeatingPin, HIGH);
    }
    else
    {
      Serial.println("OFF");
      digitalWrite(HeatingPin, LOW);
    }
  }




/*

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  Serial.println();
  Serial.println("-----------------------");
*/
}


void loop() {
  client.loop();

  if ( (unsigned long) (millis() - time1) > 60000 )
  {
    publishData(25, 21);
    time1 = millis();
  }

  /* scan i2c device */
  if ( (unsigned long) (millis() - time2) > 5000 )
  {
    /*sendi2c();*/
    time2 = millis();
  }
  /* scan i2c device end*/

  while (Serial.available())
  {
    char c = Serial.read();

    if (c == 'H')
    {
      Wire.beginTransmission(5);
      Wire.write('H');
      Serial.print("H");
      Wire.endTransmission();
    }
    else if (c == 'L')
    {
      Wire.beginTransmission(5);
      Wire.write('L');
      Serial.print("H");
      Wire.endTransmission();
    }
  }


}

















void publishData(float p_temperature, float p_humidity) {

  value = analogRead(A0);
  value = KalmanFilter.updateEstimate(value);
  vout = (value * 3.555) / 1024.0; // see text
  vin = vout / (R2 / (R1 + R2));
  vin1 = vin;//vin1 la gia tri dien ap, vin la phan tram dung luong

  vin = vin - 10.8;
  vin = (vin / 0.018);
  if (vin <= 0) {
    vin = 0;
  }
  if (vin > 100) {
    vin = 100;
  }
  // chuyen tu float vin -> kieu string
  char result[5];
  dtostrf(vin, 2, 0, result); // Leave room for too large numbers!
  client.publish("sensor_8266_01/percent", result, willQoS); //Topic name
  char result1[5];
  dtostrf(vin1, 4, 1, result1); // Leave room for too large numbers!
  client.publish("sensor_8266_01/von", result1, willQoS); //Topic name



  /*
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = vin1;//(String)p_temperature;
  root["humidity"] = vin;//(String)p_humidity;
  root.prettyPrintTo(Serial);
  Serial.println("");
  /*
     {
        "temperature": "23.20" ,
        "humidity": "43.70"
     }
  
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  //client.publish("sensor/humidity", data, true);
  client.publish("sensor/humidity", vin, true);
  yield();
  */
}
