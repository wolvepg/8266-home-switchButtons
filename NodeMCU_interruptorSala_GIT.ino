/************************* SENSOR CAPACITIVO *********************************/

#include <CapacitiveSensor.h>

/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>
#include <CapacitiveSensor.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "SSID"
#define WLAN_PASS       "PASS"

/************************* Adafruit.io Setup *********************************/


#define AIO_SERVER      "192.168.0.103" //LOCAL SERVER
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;


// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** MQTT TOPICS *********************************/

#define TOPIC_LAMPADA_SALA_PRINCIPAL "wolvegp/iot/sala/lampadaDupla"
#define TOPIC_LAMPADA_COZINHA "wolvegp/iot/cozinha/lampadaCozinha"

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish LAMPADA_SALA_PRINCIPAL_PUB = Adafruit_MQTT_Publish(&mqtt, TOPIC_LAMPADA_SALA_PRINCIPAL);
Adafruit_MQTT_Publish LAMPADA_COZINHA_PUB = Adafruit_MQTT_Publish(&mqtt, TOPIC_LAMPADA_COZINHA);

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe LAMPADA_SALA_PRINCIPAL_SUB = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_LAMPADA_SALA_PRINCIPAL);
Adafruit_MQTT_Subscribe LAMPADA_COZINHA_SUB = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_LAMPADA_COZINHA);

/*************************** INTERRUPTOR CAPACITIVO *************************/
//
//const int ledA = 3; //led indicador do interruptor
//const int ledB = 5; //led indicador do interruptor

const int emitter_cap = 5;

const int sensor_capA = 4;
const int sensor_capB = 2;

//const int ledL = 1; // valor PWM minimo para o led quando a lampada estiver desligada
const int sensibil = 100;  //varia de acordo com o resistor e tamanho da superficie capacitiva, mude para testar.

CapacitiveSensor  csA = CapacitiveSensor (emitter_cap,sensor_capA);    
CapacitiveSensor  csB = CapacitiveSensor (emitter_cap,sensor_capB);  


/************************* Variaveis Globais ********************************/
  //MARCA O TEMPO EM QUE OS BOTOES FORAM PRESSIONADOS
  //OBJETIVO EVITAR SENTIR "DOIS TOQUES"
  unsigned long tempoApress = millis();
  unsigned long tempoBpress = millis();
  long intervaloPressao = 1000;

  String estadoA = "OFF";
  String estadoB = "OFF";
//
////VARIAVEIS PARA ANIMACAO
//unsigned long animaAtempo = 0;
//unsigned long animaBtempo = 0;
//String comandoA = "nenhum";
//String comandoB = "nenhum";
//int lumiA = 1;
//int lumiB = 1;



/*************************** Sketch Code ************************************/
// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
//  Serial.begin(115200);
  delay(10);

//  Serial.println(F("Adafruit MQTT demo"));

  // Connect to WiFi access point.
//  Serial.println(); Serial.println();
//  Serial.print("Connecting to ");
//  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
  }
//  Serial.println();
//
//  Serial.println("WiFi connected");
//  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&LAMPADA_SALA_PRINCIPAL_SUB);
  mqtt.subscribe(&LAMPADA_COZINHA_SUB);

  //INTERRUPTOR SETUP
  csA.set_CS_AutocaL_Millis( 0xFFFFFFFF);
  csB.set_CS_AutocaL_Millis( 0xFFFFFFFF);

  //  pinMode(lampada, OUTPUT); 
//  pinMode(ledA, OUTPUT);
//  pinMode(ledB, OUTPUT);
//  analogWrite(ledA, ledL);
//  analogWrite(ledB, ledL);


//  
//  //VARIAVEIS PARA ANIMACAO
//  unsigned long animaAtempo = 0;
//  unsigned long animaBtempo = 0;
//  String comandoA = "nenhum";
//  String comandoB = "nenhum";
//  int lumiA = 1;
//  int lumiB = 1;
digitalWrite(LED_BUILTIN, HIGH);
}

uint32_t x=0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  

  while ((subscription = mqtt.readSubscription(50))) {
    if (subscription == &LAMPADA_SALA_PRINCIPAL_SUB) {
      estadoA = (char *)LAMPADA_SALA_PRINCIPAL_SUB.lastread;
//      Serial.print(F("Estado Lampada Sala: "));
//      Serial.println((char *)LAMPADA_SALA_PRINCIPAL_SUB.lastread);
    }

    if (subscription == &LAMPADA_COZINHA_SUB) {
      estadoB = (char *)LAMPADA_COZINHA_SUB.lastread;
//      Serial.print(F("Estado Lampada Cozinha: "));
//      Serial.println((char *)LAMPADA_COZINHA_SUB.lastread);
    }
  }
  

   //FUNCAO DOS BOTOES CAPACITIVOS
//   digitalWrite(LED_BUILTIN, HIGH);
    long capA =  csA.capacitiveSensor(240); //chama a funcao que le a capacitancia...
    long capB =  csB.capacitiveSensor(240); //chama a funcao que le a capacitancia...
//    Serial.print("\tA:");
//    Serial.print(capA);
//    Serial.print("\tB:");
//    Serial.print(capB);
//    Serial.print("\n");
    //if(capA > 10) digitalWrite(LED_BUILTIN, LOW);
    //if(capB > 20) digitalWrite(LED_BUILTIN, LOW);
    if (capA > sensibil && (millis()- tempoApress) > intervaloPressao) {
      MQTT_connect();
       long tempoInicio = millis(); //marca o tempo de inicio
        tempoApress = millis();

//        Serial.print("...");
 
        if(estadoA == "OFF") {
          if (! LAMPADA_SALA_PRINCIPAL_PUB.publish("ON")) {
//            Serial.println(F("Failed"));
          } else {
//            Serial.println(F("OK!"));
          }
        }else{
          if (! LAMPADA_SALA_PRINCIPAL_PUB.publish("OFF")) {
//            Serial.println(F("Failed"));
          } else {
//            Serial.println(F("OK!"));
          }
        }
        
    }
    
    if (capB > sensibil && (millis()- tempoBpress) > intervaloPressao) {
      MQTT_connect();
       long tempoInicio = millis(); //marca o tempo de inicio
        tempoBpress = millis();

//        Serial.print("...");
        
        if(estadoB == "OFF") {
            if (! LAMPADA_COZINHA_PUB.publish("ON")) {
//              Serial.println(F("Failed"));
            } else {
//              Serial.println(F("On send OK!"));
            }
          }else{
            if (! LAMPADA_COZINHA_PUB.publish("OFF")) {
//              Serial.println(F("Failed"));
            } else {
//              Serial.println(F("On send OK!"));
            }
            }
        
    }    

//    if (capB > sensibil && (millis()- tempoBpress) > intervaloPressao) {
//      botaoAcionado("Baa");
//    }

    //ANIMACAO DOS BOTOES
    //ANIMAÇÃO DOS BOTOES - EM INTERVALOS DE 5 MILISEGUNDOS (MÍNIMO)
//    if (comandoA != "nenhum" || comandoB != "nenhum") animacao();
//  }

  // Now we can publish stuff!
//  Serial.print(F("\nSending photocell val "));
//  Serial.print(x);
//  Serial.print("...");
//  if (! photocell.publish(x++)) {
//    Serial.println(F("Failed"));
//  } else {
//    Serial.println(F("OK!"));
//  }

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  
//  if(! mqtt.ping()) {
//    mqtt.disconnect();
//  }
  
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
//       Serial.println(mqtt.connectErrorString(ret));
//       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
//  Serial.println("MQTT Connected!");
}
