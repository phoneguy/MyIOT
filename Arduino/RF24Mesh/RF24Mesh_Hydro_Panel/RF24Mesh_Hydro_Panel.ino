/*
        RF24_Mesh_Hydro_Panel.ino
        by stevenharsanyi@gmail.com
        rev 1.0 June 11, 2017 
*/
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"

//#include <SoftwareSerial.h>
#include <EmonLib.h>
#include <SPI.h>

// Arduino NANO digital pins
#define RX_MESH_LED   4
#define TX_MESH_LED   5
#define DEBUG_LED     6
#define RF24_CE       9
#define RF24_CSN      10
#define RF24_MISO     11
#define RF24_MOSI     12
#define RF24_SCLK     13

// Arduino NANO analog pins
#define AC_VOLT_PIN    0
#define AC_CT1_PIN     1
#define AC_CT2_PIN     2

unsigned long currentMillis = 0;
long previousMillis = 0;
uint8_t update_rate    = 0;
long timer          = 0;

uint16_t update_table[7][2] = { {0,   2}, // default 2 seconds
                                {82,  1},
                                {83,  2},
                                {84,  5},
                                {85, 10},
                                {86, 30},
                                {87, 60} };
                                
// Hardware ID and Node ID
char hardware_id[7] = "HYDROIO";
uint8_t node_id = 98;

// Serial IO
char packet_one[32] = "";
char id[16];
String input_string     = "";
boolean string_complete = false;

uint8_t debug = 0;

RF24 radio(RF24_CE, RF24_CSN);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

EnergyMonitor ct1,ct2;

void setup() {
  
     // Turn mesh tx and rx leds off
    pinMode(TX_MESH_LED,       OUTPUT);
    pinMode(RX_MESH_LED,       OUTPUT);
    pinMode(DEBUG_LED,         OUTPUT);
    digitalWrite(TX_MESH_LED, HIGH);
    digitalWrite(RX_MESH_LED, HIGH);
    digitalWrite(DEBUG_LED,   HIGH);

    // Setup OpenEnergyMonitor sensors
    // emon1.voltage(AC_VOLT_PIN, 54.26, 1.7); // Voltage: input pin, calibration, phase_shift
    ct1.current(1, 88.22);  //66.22 Current: input pin, calibration. 
    ct2.current(2, 88.22);  //66.22
   
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
    
    // Set node update rate
    update_rate = update_table[0][1] * 1000;
    
}

void loop() {
  
    mesh.update();

    // V × ICT1 + V × ICT2 = V × (I1 + I3) + V × (I2 + I3) = V × I1 + V × I2 + 2 × V × I3 for north america 240v
    double irms1 = ct1.calcIrms(1480);
    double irms2 = ct2.calcIrms(1480);
    double volts = 120.00;
    double amps = irms1 + irms2;
    double int_amps = amps * 10; // ie: convert 3.2 to 32 and divide by 10 at receiving end to send decimal
    double watts = volts * amps;
    uint16_t total_amps = int_amps;
    uint16_t total_watts = watts;
    
    currentMillis = millis();    
    if(currentMillis - previousMillis >= update_rate) {
        digitalWrite(TX_MESH_LED, LOW);
        //                                   127s      32767s       32767\n
        sprintf(packet_one, "%u %u %u\n", node_id, total_watts, total_amps);
        if (debug == 1) {
        Serial.print(packet_one); 
        }
        mesh.write(&packet_one, 'S', sizeof(packet_one));
        previousMillis = currentMillis;
        digitalWrite(TX_MESH_LED, HIGH);
    }
 
// Check for incoming data from other nodes
    if(network.available()){
        RF24NetworkHeader header;
        network.peek(header);
    
    uint32_t dat  = 0;
    char data[32] = "";
    
    switch(header.type){
        case 'M':
        digitalWrite(RX_MESH_LED, LOW);
        network.read(header,&dat,sizeof(dat));
        if (debug == 1) {
        Serial.println(dat);
        }
        if (dat >= 82 && dat <= 87) {
            update_rate = update_table[dat][1] * 1000;   
            }
        else if (dat == 90) {
            digitalWrite(DEBUG_LED, HIGH);
            debug = 0;   
            }
        else if (dat == 91) {
            digitalWrite(DEBUG_LED, LOW);
            debug = 1;  
            }          
        break;
      
        case 'S':
        digitalWrite(RX_MESH_LED, LOW);
        network.read(header, &data, sizeof(data));
        Serial.print(data);
        break;
      
        default:
        digitalWrite(RX_MESH_LED, LOW);
        network.read(header,0,0);
        Serial.println(header.type);
        break;
    }
        digitalWrite(RX_MESH_LED, HIGH );
    }

} //end

