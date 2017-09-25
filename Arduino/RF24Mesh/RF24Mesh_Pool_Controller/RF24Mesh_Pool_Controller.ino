
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
#define RF24_CSN       10
#define RF24_MOSI      11
#define RF24_MISO      12
#define RF24_SCLK      13

// Arduino NANO analog pins
#define DC_VOLT_PIN    A0
#define DC_CURRENT_PIN A1
#define AC_VOLT_PIN    A2
#define AC_CURRENT_PIN A3
#define I2C_SDA_PIN    A4
#define I2C_SCL_PIN    A5
#define LED_1_PIN      A6 
#define LED_2_PIN      A7

// Number of relay channels in use
#define RELAY_CHANNELS 2
#define RELAY_ON LOW  
#define RELAY_OFF HIGH

//#define SERIAL_DEBUG
#define MESH_NOMASTER

int relay1_state = 0;
int relay2_state = 0;
int baro_state = 0;
int compass_state = 0;
int blinkm_state = 0;

int pool_temp = 0;
int air_temp = 0;
int case_temp = 0;
int board_temp = 0;
int motor_temp = 0;
int solar_temp = 0;

int voltage = 0;
int current = 0;
int dc_volt = 0;
int dc_voltage = 0;
int dc_current = 0;
int dc_watts = 0;

int ac_volt = 0;
int ac_voltage = 0;
int ac_current = 0;
int ac_watts = 0;
int ac_watts_offset = 50;

float kilowatt_hour;
float energy_cost;
float realPower;       
float apparentPower;   
float powerFActor;     
float supplyVoltage;  
float Irms;            
   
float temperature = 0;
int altitude = 0;
int humidity = 0;
int air_pressure = 0;
float pressure = 0;

int x = 0;
int y = 0;
int z = 0;
int mag_z = 0;

unsigned long currentMillis = 0;
long previousMillis = 0;
long timer1 = 0;
long timer2 = 0;
long interval = 1000;
long fast_interval = 100;
long slow_interval = 5000;
long hours = 0;
long minutes = 0;
long seconds = 0;

// Hardware ID and Node ID
char hardware_id[7] = "POOLIO";
int node_id = 99;
//byte addresses[][6] = {"1Node","2Node"};

// Serial IO
char packet_one[32] = "";
char packet_two[32] = "";
char id[32];
char data[32] = "";

String input_string = "";
boolean string_complete = false;
boolean relaystatus = false;
uint32_t displayTimer = 0;

//struct payload_t {
//  unsigned long ms;
//  unsigned long counter;
//};

int debug = 0;

RF24 radio(9,10);
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
    //hmc5883_init();
    
    // Start BlinkM
    blinkm_init();
            
    // Start ds18b20 temperature sensors
    sensors.begin();
    
    // Start DHT22 temperature and humidity sensor
    dht.begin();
    
    // Print variables header info to usb serial port
   // Serial.println("ND  PT  AT  ST  H  PR  V  C  W  R1  R2");
    
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
    //kilowatt_hour = (ac_watts * seconds) / (1000 / 3600);
    //energy_cost   = kilowatt_hour * 0.887;
    
    // energy = (ac_watts * seconds) / (1000*3600);
    
    //float calc_dcvolt    = analogRead(DC_VOLT_PIN)    * 0.00488;
    //float calc_dccurrent = analogRead(DC_CURRENT_PIN) * 0.00488;
    //dc_volt    = calc_dcvolt     * 16;
    //dc_current = calc_dccurrent  * 2;
    currentMillis = millis();    
    if(currentMillis - timer1 >= fast_interval) {   
        sensors.requestTemperatures(); // Send the command to get temperature readings
        pool_temp  = ((sensors.getTempCByIndex(0)) * 1.8 + 32);
        air_temp   = ((sensors.getTempCByIndex(1)) * 1.8 + 32);
        solar_temp = ((sensors.getTempCByIndex(2)) * 1.8 + 32);
   
        dht.readHumidity();
        dht.readTemperature();
        
        timer1 = millis();
     
        //read_compass();   
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
    float atm = pressure / 101325; // "standard atmosphere"
    float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 
    
    
        
    currentMillis = millis();    
    if(currentMillis - previousMillis >= slow_interval) {    
      
        if (blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 0, 255, 0);
            }
        // packet size is 32 bytes, split 40 bytes              1            2          3          4          5           6            7         8           9            10           11  
        // Print to hardware and software serial ports         SSS           SNNS      SNNS       SNNS       NNNS        NNNNS       NNNS        NNS       NNNNS          NS           NS
        sprintf(packet_one, "%u %i %i %i %u %u ", node_id, pool_temp, air_temp, solar_temp, humidity, air_pressure); 
        sprintf(packet_two, "%u %u %u %u %u \n", ac_voltage, ac_current, ac_watts, relay1_state, relay2_state);
        mesh.write(&packet_one, 'S', sizeof(packet_one));
        mesh.write(&packet_two, 'S', sizeof(packet_two));
      
        if (debug == 1) {  
            Serial.print(packet_one);        
            Serial.print(packet_two);
        }
        if (blinkm_state == 1) {
        blinkm_setrgb(BLINKM_ADDRESS, (ac_watts / 17), (ac_voltage), (debug * 255));
        }            
        previousMillis = millis();   
        
    }
      
    node_command();
    
}





