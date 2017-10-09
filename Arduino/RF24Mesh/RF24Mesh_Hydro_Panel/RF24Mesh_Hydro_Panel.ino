
/*
        RF24_Mesh_Hydro_Panel.ino by stevenharsanyi@gmail.com
        rev 1.0 June 11, 2017 
*/
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"

#include <SoftwareSerial.h>
#include <EmonLib.h>
#include <SPI.h>

// Arduino NANO digital pins
#define TX_MESH_LED    2
#define RX_MESH_LED    3
#define RF24_CE        9
#define RF24_CSN       10
#define RF24_MOSI      11
#define RF24_MISO      12
#define RF24_SCLK      13

// Arduino NANO analog pins
#define AC_VOLT_PIN    0
#define AC_CT1_PIN     1
#define AC_CT2_PIN     2

uint8_t ac_volt = 0;
uint8_t ac_voltage = 0;
uint8_t ac_current = 0;
uint16_t ac_watts = 0;
uint8_t ac_watts_offset = 0;

uint8_t ac_amps1 = 0;
uint8_t ac_amps2 = 0;
uint16_t ac_watts1 = 0;
uint16_t ac_watts2 = 0;

unsigned long currentMillis = 0;
long previousMillis = 0;
long timer1 = 0;
long timer2 = 0;
long interval = 1000;
long fast_interval = 500;
long slow_interval = 2000;
long hours = 0;
long minutes = 0;
long seconds = 0;

// Hardware ID and Node ID
char hardware_id[7] = "HYDROIO";
uint8_t node_id = 98;

// Serial IO
char packet_one[32] = "";

char id[32];
String input_string = "";
boolean string_complete = false;

uint8_t debug = 0;

RF24 radio(RF24_CE, RF24_CSN);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

EnergyMonitor ct1,ct2;

void setup() {
    // Setup OpenEnergyMonitor sensors
    // emon1.voltage(AC_VOLT_PIN, 54.26, 1.7); // Voltage: input pin, calibration, phase_shift
    ct1.current(1, 66.22);  // Current: input pin, calibration. 
    ct2.current(2, 66.22);
   
    // Start usb serial connections
    Serial.begin(9600);
    
    mesh.setNodeID(node_id);
   
    Serial.println("Connected To USB Serial Port");
    sprintf(id, "Hardware ID: %s", hardware_id);
    Serial.println(id);
    Serial.print("NodeID: ");
    Serial.println(mesh.getNodeID());
    
    // Now that this node has a unique ID, connect to the mesh
    mesh.begin();
               
    // Print variables header info to usb serial port
    Serial.println("NodeID Watts Amps");
    
}

void loop() {

    digitalWrite(TX_MESH_LED, LOW);
    digitalWrite(RX_MESH_LED, LOW);
    
    mesh.update();
      
    double irms1 = ct1.calcIrms(1480);
    double irms2 = ct2.calcIrms(1480);
    
    uint8_t ac_voltage = 120;
    
    ac_amps1 = irms1;
    ac_amps2 = irms2;
    
    float ac_kw1 = irms1 * 120;
    float ac_kw2 = irms2 * 120;
    
    ac_watts1 = ac_kw1;
    ac_watts2 = ac_kw2;
    
    uint8_t total_amps =2*( irms1 + irms2);
    uint16_t total_watts = 120 * (2*irms1) + 120 * (2*irms2); //( 2 * ac_voltage * irms1 + ac_voltage * irms2 ) - ac_watts_offset;
      
    currentMillis = millis();    
    if(currentMillis - previousMillis >= slow_interval) {
        digitalWrite(TX_MESH_LED, HIGH);
        //                                   97s      999999s       999\n
        sprintf(packet_one, "%u %i %i \n", node_id, total_watts, total_amps);
        if (debug == 1) {
        Serial.println(packet_one); 
        }
        mesh.write(&packet_one, 'S', sizeof(packet_one));
    previousMillis = currentMillis;
    }
    
// Check for incoming data from other nodes
    if(network.available()){
        RF24NetworkHeader header;
        network.peek(header);
    
    uint32_t dat = 0;
    char data[32] = "";
    
    switch(header.type){
        case 'M':
        digitalWrite(RX_MESH_LED, HIGH);
        network.read(header,&dat,sizeof(dat));
        if (debug == 1) {
        Serial.println(dat);
        }
        if (dat == 30) {
            debug = 0;   
           // blinkm_setrgb(BLINKM_ADDRESS, 0,0,0);
            }
        else if (dat == 31) {
            debug = 1;  
          //  blinkm_setrgb(BLINKM_ADDRESS, 255,255,0);
            }   
            
        break;
      
        case 'S':
        network.read(header, &data, sizeof(data));
        Serial.print(data);
        break;
      
        default:
        network.read(header,0,0);
        Serial.println(header.type);
        break;
    }
  }
}





