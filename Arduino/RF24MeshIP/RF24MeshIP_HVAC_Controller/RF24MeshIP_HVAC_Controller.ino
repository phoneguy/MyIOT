
/*
        HVAC Controller by stevenharsanyi@gmail.com
        rev 1.0 September 25, 2017 
*/
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"

#include <SoftwareSerial.h>
//#include <EmonLib.h>
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include "cactus_io_DHT22.h"

// I2C addresses for devices
#define HMC5883_ADDRESS 0x1E 
#define BMP085_ADDRESS  0x77
#define BLINKM_ADDRESS  0x09
#define ITG3200_ADDRESS 0x69
#define BMA180_ADDRESS  0x40
#define MPU6050_ADDRESS 0x68 // 0x69
#define ADXL345_ADDRESS 0x53
#define MS5611_ADDRESS  0xEF // 0xED

// Arduino NANO digital pins
#define RELAY1_PIN     2
#define RELAY2_PIN     3
#define DHT22_PIN      4   
#define ONEWIREBUS_PIN 5
#define RF24_CE        9
#define RF24_CSN       10
#define RF24_MOSI      11
#define RF24_MISO      12
#define RF24_SCLK      13

// Arduino NANO analog p
#define I2C_SDA_PIN    A4
#define I2C_SCL_PIN    A5
#define LED_1_PIN      A6 
#define LED_2_PIN      A7

// Number of relay channels in use
#define RELAY_CHANNELS 2
#define RELAY_ON LOW  
#define RELAY_OFF HIGH

uint8_t relay1_state = 0;
uint8_t relay2_state = 0;
uint8_t baro_state = 0;
uint8_t compass_state = 0;
uint8_t blinkm_state = 0;

int air_return = 0;
int air_temp = 0;
int case_temp = 0;
int board_temp = 0;
int motor_temp = 0;
int air_supply = 0;
int air_inside = 0;
uint16_t humidity_inside = 0;
uint16_t pressure_inside = 0;

float temperature = 0;
int altitude = 0;
uint16_t humidity = 0;
uint16_t air_pressure = 0;
float pressure = 0;

unsigned long currentMillis = 0;
long previousMillis = 0;
long timer1 = 0;
long timer2 = 0;
long interval = 1000;
long fast_interval = 100;
long slow_interval = 5000;

// Hardware ID and Node ID
char hardware_id[7] = "HVACIO";
uint8_t node_id = 91;

// Serial IO
char packet_one[32] = "";
char packet_two[32] = "";
char id[32];
char data[32] = "";

String input_string = "";
boolean string_complete = false;

uint8_t debug = 0;

RF24 radio(9,10);
RF24Network network(radio);
RF24Mesh mesh(radio, network);


// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONEWIREBUS_PIN); 

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Setup DHT22 sensor
DHT22 dht(DHT22_PIN);

void setup() {
    
    // Setup relay pins
    pinMode(RELAY1_PIN,      OUTPUT);
    pinMode(RELAY2_PIN,      OUTPUT);
    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, HIGH);
   
    // Start usb serial connections
    Serial.begin(9600);
   
    mesh.setNodeID(99);
    
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
            
    // Start ds18b20 temperature sensors
    sensors.begin();
    
    // Start DHT22 temperature and humidity sensor
    dht.begin();
       
}

void loop() {

    mesh.update();
    
    currentMillis = millis();    
    if(currentMillis - timer1 >= fast_interval) {   
        sensors.requestTemperatures(); // Send the command to get temperature readings
        air_return  = ((sensors.getTempCByIndex(0)) * 1.8 + 32);
        air_supply  = ((sensors.getTempCByIndex(1)) * 1.8 + 32);
        air_inside  = ((sensors.getTempCByIndex(2)) * 1.8 + 32);
   
        dht.readHumidity();
        dht.readTemperature();
        
        timer1 = millis();
    }
    
    humidity  = dht.humidity;
    case_temp = dht.temperature_F;
        
    if (baro_state == 1) {
       currentMillis = millis();    
       if(currentMillis - timer2 >= interval) {
        temperature  = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
        pressure     = bmp085GetPressure(bmp085ReadUP());
       }
        timer2 = millis();   
    }
    
    air_pressure = pressure * 0.01;
    board_temp   = temperature;
    float atm = pressure / 101325; 
    float altitude = calcAltitude(pressure);
    
    if (blinkm_state == 1) {
//        blinkm_setrgb(BLINKM_ADDRESS, (ac_watts / 17), (ac_voltage), (debug * 255));
        }       
                
    currentMillis = millis();    
    if(currentMillis - previousMillis >= slow_interval) {    
      
        if (blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 0, 255, 0);
            }
        // packet size is 32 bytes, split 40 bytes              1            2          3          4          5           6            7         8           9            10           11  
        // Print to hardware and software serial ports         SSS           SNNS      SNNS       SNNS       NNNS        NNNNS       NNNS        NNS       NNNNS          NS           NS
        sprintf(packet_one, "%u %i %i %i %u %u ", node_id, air_return, air_supply, air_temp, humidity, air_pressure); 
        sprintf(packet_two, "%u %u %u %u %u \n", air_inside, humidity_inside, pressure_inside, relay1_state, relay2_state);
        mesh.write(&packet_one, 'S', sizeof(packet_one));
        mesh.write(&packet_two, 'S', sizeof(packet_two));
      
        if (debug == 1) {  
            Serial.print(packet_one);        
            Serial.print(packet_two);
        }
             
        previousMillis = millis();   
        
    }
      
    node_command();
    
}





