

/*
        RF24Mesh_GreenHouse_Controller.ino by stevenharsanyi
        rev 1.0 October 1,2017 
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

// Arduino mega2560 digital pins
#define RELAY1_PIN     2
#define RELAY2_PIN     3
#define DHT22_PIN      4   
#define ONEWIREBUS_PIN 5
#define RELAY3_PIN     6
#define RELAY4_PIN     7
#define RELAY5_PIN     8
#define RELAY6_PIN     9
#define RELAY7_PIN     10
#define RELAY8_PIN     11
#define RX_MESH_LED     14 
#define TX_MESH_LED     15
#define DEBUG_LED       16 

// Arduino mega2560 analog pins
#define AC_VOLT_PIN          A0
#define AC_CURRENT_PIN       A1
#define WATERTANK_LOW_PIN    A2
#define WATERTANK_FULL_PIN   A3
#define I2C_SDA_PIN          A4
#define I2C_SCL_PIN          A5
#define DEHUMIDTANK_LOW      A6
#define DEHUMIDTANK_FULL     A7

// Number of relay channels in use
#define RELAY_CHANNELS 8
#define RELAY_ON LOW  
#define RELAY_OFF HIGH

// Arduino Digital I/O pin numbers for UNO R3
//enum { Relay1=2, Relay2=3, Relay3=6, Relay4=7, Relay5=8, Relay6=9,
//       Relay7=10,   Relay8=11};

// Number of relays in the array
//enum { maxRelayCount = sizeof relays / sizeof relays[0] };
//enum { RELAY_OFF = HIGH };  // Set LOW or HIGH as appropriate

//uint8_t relaystate[8] = {0,0,0,0,0,0,0,0};


uint8_t relay_state  = 0;
uint8_t relay1_state = 0;
uint8_t relay2_state = 0;
uint8_t relay3_state = 0;
uint8_t relay4_state = 0;
uint8_t relay5_state = 0;
uint8_t relay6_state = 0;
uint8_t relay7_state = 0;
uint8_t relay8_state = 0;
uint8_t relays[9][3] = { {0, 0, 0},
                     {1, RELAY1_PIN, 0},
                     {2, RELAY2_PIN, 0},
                     {3, RELAY3_PIN, 0},
                     {4, RELAY4_PIN, 0},
                     {5, RELAY5_PIN, 0},
                     {6, RELAY6_PIN, 0},
                     {7, RELAY7_PIN, 0},
                     {8, RELAY8_PIN, 0} 
                     };
uint8_t baro_state   = 0;

uint8_t blinkm_state = 0;

int8_t flower_temp      = 0;
int8_t grow_temp        = 0;
int8_t watertank_temp   = 0;
int8_t exhaust_temp     = 0;
int8_t outside_temp     = 0;
int8_t flowerlight_temp = 0;

uint8_t voltage = 0;
uint8_t current = 0;

uint8_t ac_volt = 0;
uint8_t ac_voltage = 0;
uint8_t ac_current = 0;
uint8_t ac_watts = 0;
uint16_t ac_watts_offset = 50;

float realPower;       
float apparentPower;   
float powerFActor;     
float supplyVoltage;  
float Irms;            
   
float temperature = 0;
int altitude = 0;
uint8_t humidity = 0;
uint16_t air_pressure = 0;
float pressure = 0;

unsigned long currentMillis = 0;
long previousMillis = 0;
long timer1 = 0;
long timer2 = 0;
long interval = 1000;
long fast_interval = 100;
long slow_interval = 5000;
long two_seconds = 2000;
uint8_t update_rate    = 0;
//                           command  seconds
uint16_t update_table[7][2] = { {0, 10}, // default 10 seconds
                               {82,  1},
                               {82,  2},
                               {83,  5},
                               {84, 10},
                               {85, 30},
                               {86, 60} };
// Hardware ID and Node ID
char hardware_id[7] = "GREENIO";
uint8_t node_id = 96;

// Serial IO
char packet_one[32] = "";
char packet_two[32] = "";
char id[32];
char data[32] = "";

String input_string = "";
boolean string_complete = false;

uint8_t debug = 1;

RF24 radio(53,48);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

EnergyMonitor emon1;                   // Create an instance

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONEWIREBUS_PIN); 

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Setup DHT22 sensor
DHT22 dht(DHT22_PIN);

void setup() {
   
    // Set pins to OFF & declare pins as OUTPUTS
    for(int i = 1; i <= RELAY_CHANNELS; ++i) {
    pinMode(relays[i][1], OUTPUT);
    digitalWrite(relays[i][1], RELAY_OFF);
    }
    
    // Setup OpenEnergyMonitor sensors
    emon1.voltage(AC_VOLT_PIN, 56.13, 1.7); // Voltage: input pin, calibration, phase_shift
    emon1.current(AC_CURRENT_PIN, 228.22);  // Current: input pin, calibration. 

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
    
    // Start compass
    //hmc5883_init();
    
    // Start BlinkM
    blinkm_init();
            
    // Start ds18b20 temperature sensors
    sensors.begin();
    
    // Start DHT22 temperature and humidity sensor
    dht.begin();
    
    // Set node update rate
    update_rate = update_table[0][1] * 1000;
           
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
    if(currentMillis - timer1 >= two_seconds) {   
        sensors.requestTemperatures(); // Send the command to get temperature readings
        watertank_temp  = ((sensors.getTempCByIndex(0)) * 1.8 + 32);
        exhaust_temp     = ((sensors.getTempCByIndex(1)) * 1.8 + 32);
        flowerlight_temp = ((sensors.getTempCByIndex(2)) * 1.8 + 32);
   
        dht.readHumidity();
        dht.readTemperature();
 humidity  = dht.humidity;
    outside_temp = dht.temperature_F;
        relay1_state = relays[1][2];
        relay2_state = relays[2][2];
        relay3_state = relays[3][2];
        relay4_state = relays[4][2];
        relay5_state = relays[5][2];
        relay6_state = relays[6][2];
        relay7_state = relays[7][2];
        relay8_state = relays[8][2];

        timer1 = millis();
     
    }
    
   
        
    if (baro_state == 1) {
       currentMillis = millis();    
       if(currentMillis - timer2 >= interval) {
        temperature  = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
        pressure     = bmp085GetPressure(bmp085ReadUP());
       }
        timer2 = millis();   
    }
    
    air_pressure = pressure * 0.01;
    grow_temp   = temperature;
    float atm = pressure / 101325; 
    float altitude = calcAltitude(pressure);
    
    if (blinkm_state == 1) {
        blinkm_setrgb(BLINKM_ADDRESS, (ac_watts / 17), (ac_voltage), (debug * 255));
        }       
                
    currentMillis = millis();    
    if(currentMillis - previousMillis >= slow_interval) {    
      
        if (blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 0, 255, 0);
            }
        // packet size is 32 bytes  
        //                                          -99s        -999s     -999s           -999s        -999s         -999s             
        sprintf(packet_one, "%u %i %i %i %i %i ", node_id, flower_temp, grow_temp, watertank_temp, exhaust_temp, outside_temp); 
        //                                                      -999s             999s           1s            1s             1s            1s           1s             1s             1s          1
        sprintf(packet_two, "%i %u %i %i %i %i %i %i %i %i\n", flowerlight_temp, air_pressure, relay1_state, relay2_state, relay3_state, relay4_state, relay5_state, relay6_state, relay7_state, relay8_state );
        mesh.write(&packet_one, 'S', sizeof(packet_one));
        mesh.write(&packet_two, 'S', sizeof(packet_two));
      
        if (debug == 1) {  
            Serial.print(packet_one);        
            Serial.print(packet_two);
        }
             
        previousMillis = millis();   
        
    }
      
    node_command();
    if (debug == 1) {
      serial_command();
    }

    
}





