
/*
        HVAC Controller by stevenharsanyi@gmail.com
        rev 1.0 September 25, 2017 
*/
#include "DigitalIO.h"

#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"

//#include <SoftwareSerial.h>
//#include <EmonLib.h>
#include <SPI.h>
#include <Wire.h>
//#include <OneWire.h> 
#include <DallasTemperature.h>
#include "cactus_io_DHT22.h"

// I2C addresses for devices
#define HMC5883_ADDRESS 0x1E 
#define BMP085_ADDRESS  0x77
#define BLINKM_ADDRESS  0x09
#define ITG3200_ADDRESS 0x68
#define BMA180_ADDRESS  0x40
#define MPU6050_ADDRESS 0x69 // 0x69
#define ADXL345_ADDRESS 0x53
#define MS5611_ADDRESS  0xEF // 0xED

// Arduino NANO digital pins
#define DHT22_PIN       2
#define ONEWIREBUS_PIN  3
#define RX_MESH_LED     4 
#define TX_MESH_LED     5
#define DEBUG_LED       6 
#define RELAY1_PIN      7
#define RELAY2_PIN      8
#define SPI_CE         9
#define SPI_CSN       10
#define SPI_MISO      11
#define SPI_MOSI      12
#define SPI_SCLK      13
//const uint8_t SOFT_SPI_MISO_PIN = 4;
//const uint8_t SOFT_SPI_MOSI_PIN = 5;
//const uint8_t SOFT_SPI_SCK_PIN  = 6;
//const uint8_t SPI_MODE = 0;
// Arduino NANO analog pins
#define PHOTORESISTOR  A0
#define I2C_SDA_PIN    A4
#define I2C_SCL_PIN    A5

// Arduino MEGA 2560 pins
#define MEGA_SPI_CSN  48
#define MEGA_SPI_MISO 50
#define MEGA_SPI_MOSI 51
#define MEGA_SPI_SCK  52
#define MEGA_SPI_CE   53

//
uint8_t rgbled[4][3] = {{0, 0,           0},
                        {1, RX_MESH_LED, 0},
                        {2, TX_MESH_LED, 0},
                        {3, DEBUG_LED,   0}};
                         
// Number of relay channels in use
#define RELAY_CHANNELS 2
#define RELAY_ON LOW  
#define RELAY_OFF HIGH

uint8_t relay1_state  = 0;
uint8_t relay2_state  = 0;
//              relay#, pin, state         
uint8_t relays[3][3] = { {0, 0, 0},
                         {1, RELAY1_PIN, 0},
                         {2, RELAY2_PIN, 0} };

#define baro    0
#define blinkm  0
#define compass 0
uint8_t baro_state    = 0;
uint8_t blinkm_state  = 0;
uint8_t compass_state = 0;
float pressure        = 0;
float temperature     = 0;
int air_temp          = 0;
uint16_t humidity     = 0;
uint16_t air_pressure = 0;
uint16_t photo_resistor  = 0;

unsigned long currentMillis = 0;
long previousMillis = 0;
long timer1         = 0;
long timer2         = 0;
uint16_t one_hz         = 1000;
uint16_t two_hz         = 500;
uint16_t update_rate    = 0;

//                           command  seconds
uint16_t update_table[7][2] = { {0, 10}, // default 30 seconds
                               {82,  1},
                               {82,  2},
                               {83,  5},
                               {84, 10},
                               {85, 30},
                               {86, 60} };
                               
// Hardware ID and Node ID
char hardware_id[10] = "INFONODEIO";
uint8_t node_id = 91;

// Serial IO
char packet_one[32] = "";
char packet_two[32] = "";
char id[32];
char data[32] = "";

String input_string = "";
boolean string_complete = false;

uint8_t debug  = 1;
uint32_t timer = 0;

#define LOW 0
#define HIGH 1

/***** Configure the chosen CE,CS pins *****/
RF24 radio(SPI_CE, SPI_CSN);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
//OneWire oneWire(ONEWIREBUS_PIN); 

// Pass our oneWire reference to Dallas Temperature. 
//DallasTemperature sensors(&oneWire);

// Setup DHT22 sensor
DHT22 dht(DHT22_PIN);

void setup() {
pinMode(10,OUTPUT);
digitalWrite(10, HIGH);
pinMode(13,OUTPUT);
digitalWrite(13,HIGH);
    // Setup relay pins
    for(int i = 1; i <= RELAY_CHANNELS; ++i) {
    pinMode(relays[i][1], OUTPUT);
    digitalWrite(relays[i][1], HIGH);
    }
    
    // Setup rgb led
    for(int i = 1; i <= 3; ++i) {
    pinMode(rgbled[i][1], OUTPUT);
    digitalWrite(rgbled[i][1], HIGH);
    }
    
    // Start usb serial connections
    Serial.begin(9600);
   
    mesh.setNodeID(node_id);
    
    sprintf(id, "Hardware ID: %s", hardware_id);
    Serial.println(id);
    Serial.print("NodeID: ");
    Serial.println(mesh.getNodeID());
    
    // Now that this node has a unique ID, connect to the mesh
    mesh.begin();

    // Join the i2c bus
    Wire.begin();
    
    scan_i2cbus();
 
    // Start barometer
    bmp085_init();
        
    // Start BlinkM
    blinkm_init();
      
    // Start DHT22 temperature and humidity sensor
    dht.begin();

    // Set node update rate
    update_rate = update_table[0][1] * 1000;
    Serial.print("Default update rate set to "); Serial.print(update_rate / 1000); Serial.println(" seconds"); 
}

void loop() {
        
    mesh.update();
    
    node_command();

    if (debug == 1) {
        if(millis() - timer > 10000){
        timer = millis();
        Serial.println(" ");
        Serial.println(F("********Assigned Addresses********"));
        for(int i=0; i<mesh.addrListTop; i++){
        Serial.print("NodeID: ");
        Serial.print(mesh.addrList[i].nodeID);
        Serial.print(" RF24Network Address: 0");
        Serial.println(mesh.addrList[i].address,OCT);
        }
        Serial.println(F("**********************************"));
        }
    }  

    currentMillis = millis();    
    if(currentMillis - timer1 >= one_hz) {   
        
        dht.readHumidity();
        dht.readTemperature();

        humidity  = dht.humidity;
        air_temp = dht.temperature_F;
        photo_resistor = analogRead(PHOTORESISTOR) / 10;
        timer1 = millis();
    }
        
    if (baro == 1 && baro_state == 1) {
        currentMillis = millis();    
        if(currentMillis - timer2 >= two_hz) {
        temperature  = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
        pressure     = bmp085GetPressure(bmp085ReadUP());

        air_pressure = pressure * 0.01;
            
        timer2 = millis(); 
        
        }
    }
    
    if (blinkm == 1 && blinkm_state == 1) {
        blinkm_setrgb(BLINKM_ADDRESS, air_temp, humidity, photo_resistor);
        }       
                
    currentMillis = millis();    
    if(currentMillis - previousMillis >= update_rate) {    
        digitalWrite(TX_MESH_LED, LOW);
        if (blinkm == 1 && blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 0, 255, 0);
            }
        // packet size is 32 bytes max
        sprintf(packet_one, "%u %i %i %u", node_id, air_temp, humidity, photo_resistor); 
        sprintf(packet_two, "%u %u \n",  relay1_state, relay2_state);
        mesh.write(&packet_one, 'S', sizeof(packet_one));
        mesh.write(&packet_two, 'S', sizeof(packet_two));

        if (debug == 1) {
          
            Serial.print(packet_one);        
            Serial.print(packet_two);
        }
             
        previousMillis = millis();   
        digitalWrite(TX_MESH_LED, HIGH);
    }
      
} //END



