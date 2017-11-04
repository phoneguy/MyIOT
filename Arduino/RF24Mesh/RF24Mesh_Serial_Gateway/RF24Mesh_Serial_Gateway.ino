  /*
  * RF24Mesh_Serial_Gateway
  * stevenharsanyi@gmail.com
  * September 20, 2017 v1.0
  */
  
  
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>

// Arduino NANO digital pins
#define RX_MESH_LED 4
#define TX_MESH_LED 5
#define DEBUG_LED   6 
#define RF24_CE     9
#define RF24_CSN    10
#define RF24_MISO   11
#define RF24_MOSI   12
#define RF24_SCLK   13

// Arduino MEGA 2560 pins
#define MEGA_RF24_CE  53
#define MEGA_RF24_CSN 48

#define LOW 0
#define HIGH 1

/***** Configure the chosen CE,CS pins *****/
RF24 radio(RF24_CE, RF24_CSN);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

uint32_t timer = 0;
uint8_t debug =  0;
boolean string_complete = false;
String input_string = "";
char packet_one[32] = "";
 
void setup() {
  
    Serial.begin(9600);
   
    // Set the nodeID to 0 for the master node
    mesh.setNodeID(0);
    Serial.println("Hardware ID: RF24MESHGATEWAY");
    Serial.print("Node ID: ");
    Serial.println(mesh.getNodeID());
    
    // Connect to the mesh
    mesh.begin();
    
    // Turn mesh tx and rx leds off
    pinMode(TX_MESH_LED,      OUTPUT);
    pinMode(RX_MESH_LED,      OUTPUT);
    digitalWrite(TX_MESH_LED, HIGH);
    digitalWrite(RX_MESH_LED, HIGH);
}

void loop() {    
     
    // Call mesh.update to keep the network updated
    mesh.update();
   
    // In addition, keep the 'DHCP service' running on the master node so addresses will
    // be assigned to the sensor nodes
    mesh.DHCP();
  
    // Check for incoming data from the sensors
    if(network.available()){
        RF24NetworkHeader header;
        network.peek(header);
    
        uint32_t dat = 0;
        char data[32] = "";
    
        switch(header.type){
            case 'M':
            digitalWrite( RX_MESH_LED, LOW);
            network.read(header,&dat,sizeof(dat));
            Serial.println(dat);
            break;
      
            case 'S':
            digitalWrite( RX_MESH_LED, LOW);
            network.read(header, &data, sizeof(data));
            Serial.print(data);
            break;
      
            default:
            digitalWrite( RX_MESH_LED, LOW);
            network.read(header,0,0);
            Serial.println(header.type);
            break;
        }
        digitalWrite( RX_MESH_LED, HIGH);
        
        serial_event();
    }
    
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

}

// Read from serial port and send node id and command
static void serial_event() {

   while (Serial.available()) {
        char rx_character = (char)Serial.read();
        input_string += rx_character;

        if (rx_character == '\n') {
        string_complete = true;
        digitalWrite( TX_MESH_LED, LOW);
        }
    }  
        // 2 digit node id and 2 digit command
        uint8_t rx_nodeid = 0;
        uint16_t tmp = 0;
        uint16_t rx_command = 0;
        tmp = input_string.toInt();
        rx_nodeid = tmp / 100;
        rx_command = tmp - (rx_nodeid * 100);
        
        if (rx_nodeid > 0) {
            mesh.write(&rx_command, 'M', sizeof(rx_command), rx_nodeid);
            if (debug == 1 ) {
            Serial.print("Node ID: "); Serial.print(rx_nodeid); Serial.print(" Command: "); Serial.println(rx_command);
            }
            digitalWrite( TX_MESH_LED, HIGH);
        }
        
    input_string = "";
    string_complete = false;
    
}


