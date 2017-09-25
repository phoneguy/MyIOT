
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
#define RF24_CE        9
#define RF24_CSN       10
#define RF24_MISO      12
#define RF24_MOSI      11
#define RF24_SCLK      13

// Arduino NANO analog pins
#define AC_VOLT_PIN    A0
#define AC_CT1_PIN     1
#define AC_CT2_PIN     2

int ac_volt = 0;
int ac_voltage = 0;
int ac_current = 0;
int ac_watts = 0;
int ac_watts_offset = 0;

int ac_amps1 = 0;
int ac_amps2 = 0;
int ac_watts1 = 0;
int ac_watts2 = 0;

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
int node_id = 98;

// Serial IO
char packet_one[32] = "";

char id[32];
String input_string = "";
boolean string_complete = false;

int debug = 0;

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
    
    mesh.update();
      
    double irms1 = ct1.calcIrms(1480);
    double irms2 = ct2.calcIrms(1480);
    
    int ac_voltage = 120;
    
    ac_amps1 = irms1;
    ac_amps2 = irms2;
    
    float ac_kw1 = irms1 * 120;
    float ac_kw2 = irms2 * 120;
    
    ac_watts1 = ac_kw1;
    ac_watts2 = ac_kw2;
    
    int total_amps =2*( irms1 + irms2);
    int total_watts = 120 * (2*irms1) + 120 * (2*irms2); //( 2 * ac_voltage * irms1 + ac_voltage * irms2 ) - ac_watts_offset;
      
    currentMillis = millis();    
    if(currentMillis - previousMillis >= slow_interval) {
     
        sprintf(packet_one, "%u %i %i \n", node_id, total_watts, total_amps);
        if (debug == 1) {
        Serial.println(packet_one); 
        }
        mesh.write(&packet_one, 'S', sizeof(packet_one));
    previousMillis = currentMillis;
    }
// Check for incoming data from the sensors
    if(network.available()){
        RF24NetworkHeader header;
        network.peek(header);
    
    uint32_t dat=0;
    char data[32]="";
    
    switch(header.type){
        // Display the incoming millis() values from the sensor nodes
        case 'M':
        network.read(header,&dat,sizeof(dat));
        if (debug == 1) {
        Serial.println(dat);
        }
        if (dat == 930) {
            debug = 0;   
           // blinkm_setrgb(BLINKM_ADDRESS, 0,0,0);
            }
        else if (dat == 931) {
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





