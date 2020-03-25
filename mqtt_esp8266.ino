#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h> //include Wire.h library
SimpleKalmanFilter KalmanFilter(1, 1, 0.01);




int analogInput = 0;
float vout = 0.0;
float vin = 0.0;
float R1 = 100000.0; // resistance of R1 (100K) -see text!
float R2 = 10000.0; // resistance of R2 (10K) - see text!
int value = 0;



unsigned long time1 = 0;
unsigned long time2 = 0;

const char* ssid = "home"; // Enter your WiFi name
const char* password =  "0985328757"; // Enter WiFi password
const char* mqttServer = "192.168.1.10";
const int mqttPort = 1883;
const char* mqttUser = "sunnyhome";
const char* mqttPassword = "sum";

byte willQoS = 1;                          //qos of publish (see README)

/* khai bao dc nhan I2C */
int slaveAddress = 0x7E;


WiFiClient espClient;
PubSubClient client(espClient);


void read_von() {
  value = analogRead(analogInput);
  value = KalmanFilter.updateEstimate(value);
  vout = (value * 3.555) / 1024.0; // see text
  vin = vout / (R2 / (R1 + R2));
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
  client.publish("home/solar/robot", result, willQoS); //Topic name
}




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

    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }
  client.publish("home/solar/robot", "ESP8266-solarclean-ready"); //Topic name
  client.subscribe("home/control/solar/robot", 1);
  /* scan I2C device */
  Wire.begin();
}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  Serial.println();
  Serial.println("-----------------------");

}

void loop() {
  client.loop();
  if ( (unsigned long) (millis() - time1) > 60000 )
  {
    read_von();
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
