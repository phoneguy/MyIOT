
/*
        PoolController.ino by stevenharsanyi@gmail.com
        rev 1.0 June 11, 2017 
*/
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"

#include <SoftwareSerial.h>
#include <EmonLib.h>
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
#define RF24_CSN      10
#define RF24_MISO     11
#define RF24_MOSI     12
#define RF24_SCLK     13

// Arduino NANO analog pins
#define AC_VOLT_PIN    A2
#define AC_CURRENT_PIN A3
#define I2C_SDA_PIN    A4
#define I2C_SCL_PIN    A5

// Number of relay channels in use
#define RELAY_CHANNELS 2
#define RELAY_ON LOW  
#define RELAY_OFF HIGH

// Enable or disable devices
#define compass 0
#define baro    1
#define blinkm  1

// Variables
uint8_t compass_state = 0;
uint8_t baro_state    = 0;
uint8_t blinkm_state  = 0;

uint8_t max_relays    = (RELAY_CHANNELS * 10) + 1; // control msg is relay and state ie: relay 1 state 0 is 10
uint8_t min_relays    = (RELAY_CHANNELS - (RELAY_CHANNELS - 1)) * 10;
uint8_t relay1_state  = 0;
uint8_t relay2_state  = 0;
//              relay#, pin, state         
uint8_t relays[3][3] = { {0, 0, 0},
                         {1, 2, 0},
                         {2, 3, 0} };
uint8_t rx_command = 0;
uint8_t rx_state   = 0;
uint8_t relay      = 0;
uint8_t relay_pin  = 0;

int air_temp   = 0;
int board_temp = 0;
int case_temp  = 0;
int pool_temp  = 0;
int solar_temp = 0;

uint8_t ac_current       = 0;
uint16_t ac_volts        = 0;
uint16_t ac_voltage      = 0;
uint16_t ac_watts        = 0;
uint16_t ac_watts_offset = 50;

float realPower;       
float apparentPower;   
float powerFActor;     
float supplyVoltage;  
float Irms;            
   
float temperature     = 0;
uint8_t humidity      = 0;
uint16_t air_pressure = 0;
float pressure        = 0;

int x = 0;
int y = 0;
int z = 0;

// Timers
unsigned long currentMillis = 0;
long timer1  = 0;
long timer2  = 0;
long one_hz  = 1000;
long ten_hz  = 100;
long update_rate = 5000; //     command  seconds
uint16_t update_table[7][2] = { {0,   5}, // default 5 seconds
                                {82,  1},
                                {82,  2},
                                {83,  5},
                                {84, 10},
                                {85, 30},
                                {86, 60} };
                                
// Hardware ID and Node ID
char hardware_id[7] = "POOLIO";
uint8_t node_id = 99;

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

EnergyMonitor emon1;

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
    
    // Setup OpenEnergyMonitor sensors
    emon1.voltage(AC_VOLT_PIN, 56.13, 1.7); // Voltage: input pin, calibration, phase_shift
    emon1.current(AC_CURRENT_PIN, 228.22);  // Current: input pin, calibration. 

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
    
    // Start compass
    hmc5883_init();
    
    // Start BlinkM
    blinkm_init();
            
    // Start ds18b20 temperature sensors
    sensors.begin();
    
    // Start DHT22 temperature and humidity sensor
    dht.begin();
       
}

void loop() {

    mesh.update();

    emon1.calcVI(20,2000);                       // Calculate all. No.of half wavelengths (crossings), time-out
    realPower       = emon1.realPower;     // extract Real Power into variable
    apparentPower   = emon1.apparentPower; // extract Apparent Power into variable
    powerFActor     = emon1.powerFactor;   // extract Power Factor into Variable
    supplyVoltage   = emon1.Vrms;          // extract Vrms into Variable
    Irms            = emon1.Irms;          // extract Irms into Variable
    
    ac_voltage = supplyVoltage; // integer
    ac_current = Irms;          // integer
    ac_watts   = (supplyVoltage * Irms) - ac_watts_offset; // integer
    
    currentMillis = millis();    
    if(currentMillis - timer1 >= one_hz) {   
        sensors.requestTemperatures(); // Send the command to get temperature readings
        pool_temp  = ((sensors.getTempCByIndex(0)) * 1.8 + 32);
        air_temp   = ((sensors.getTempCByIndex(1)) * 1.8 + 32);
        solar_temp = ((sensors.getTempCByIndex(2)) * 1.8 + 32);
               
        dht.readHumidity();
        dht.readTemperature();
        humidity  = dht.humidity;
        case_temp = dht.temperature_F;

        if (baro == 1 && baro_state == 1) {
        temperature  = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
        pressure     = bmp085GetPressure(bmp085ReadUP());
        }
        air_pressure = pressure * 0.01;
        board_temp   = temperature;
    
        if (compass == 1 && compass_state == 1) {
        read_compass();  
        } 

        timer1 = millis();

    }
   
    if (blinkm == 1 && blinkm_state == 1) {
        blinkm_setrgb(BLINKM_ADDRESS, (ac_watts / 17), (ac_voltage), (debug * 255));
    }       
                
    currentMillis = millis();    
    if(currentMillis - timer2 >= update_rate) {    
      
        if (blinkm == 1 && blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 0, 255, 0);
            }
        relay1_state = relays[1][2];
        relay2_state = relays[2][2];
        
        // packet size is 32 bytes, split 40 bytes              1            2          3          4          5           6            7         8           9            10           11  
        // Print to hardware and software serial ports         SSS           SNNS      SNNS       SNNS       NNNS        NNNNS       NNNS        NNS       NNNNS          NS           NS
        sprintf(packet_one, "%u %i %i %i %u %u ", node_id, pool_temp, air_temp, solar_temp, humidity, air_pressure); 
        sprintf(packet_two, "%u %u %u %u %u \n", ac_voltage, ac_current, ac_watts, relay1_state, relay2_state);
        mesh.write(&packet_one, 'S', sizeof(packet_one));
        mesh.write(&packet_two, 'S', sizeof(packet_two));
      
        if (debug == 1) {  
            Serial.print(packet_one);        
            Serial.print(packet_two);

            if ( compass == 1 && compass_state == 1) {
                // Print out compass values for each axis
                Serial.print("x: ");
                Serial.print(x);
                Serial.print("  y: ");
                Serial.print(y);
                Serial.print("  z: ");
                Serial.println(z);
                }
        }
             
        timer2 = millis();   
        
    }
      
    node_command();
    
} //end





