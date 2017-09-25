 
 /*
  * RF24Mesh Sensor network serial gateway
  * stevenharsanyi@gmail.com
  * September 20, 2017 v1.0
  */
  
  
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>

/***** Configure the chosen CE,CS pins *****/
RF24 radio(53,48);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

uint32_t displayTimer = 0;
uint8_t debug = 0;
boolean string_complete = false;
String input_string = "";
uint8_t nodeID;
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
    mesh.setStaticAddress(99, 02);
    mesh.setStaticAddress(98, 03);
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
    
    uint32_t dat=0;
    char data[32]="";
    
    switch(header.type){
        // Display the incoming millis() values from the sensor nodes
        case 'M':
        network.read(header,&dat,sizeof(dat));
        Serial.println(dat);
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
    delay(1);
    serial_event();
  }
    
  if (debug == 1) {
      if(millis() - displayTimer > 10000){
    displayTimer = millis();
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


static void serial_event() {

   while (Serial.available()) {
        char rx_character = (char)Serial.read();
        input_string += rx_character;

        if (rx_character == '\n') {
        string_complete = true;
        }
    }  
        // 2 digit node id and 2 digit command
        int rx_nodeid = 0;
        int tmp = 0;
        int rx_command = 0;
        tmp = input_string.toInt();
        rx_nodeid = tmp / 100;
        rx_command = tmp - (rx_nodeid * 100);
        if (rx_nodeid > 0) {
            mesh.write(&rx_command, 'M', sizeof(rx_command), rx_nodeid);
        }
        if (debug == 1) {
          Serial.print("Node ID: "); Serial.print(rx_nodeid); Serial.print(" Command: "); Serial.println(rx_command);
        }
   /* if(input_string == "99 10") {
        if (debug == 1) {
            Serial.print("R1 OFF\n"); // Print to USB port
            }
        int tmp = 910;
        mesh.write(&tmp, 'M', sizeof(tmp),99);
        
        }
    else if (input_string == "99 11") {
        if (debug == 1) {
            Serial.print("R1 ON\n");
            }
        int tmp = 911;
        mesh.write(&tmp, 'M', sizeof(tmp),99);
        
        }
    else if (input_string == "99 20") {
        //Serial.print("R2 OFF\n");
        int tmp = 920;
        mesh.write(&tmp, 'M', sizeof(tmp),99);
        
        }
    else if (input_string == "99 21") {
        //Serial.print("R2 ON\n");
        int tmp = 921;
        mesh.write(&tmp, 'M', sizeof(tmp),99);
        
        }
    else if (input_string == "99 30") {
        debug = 0;   
        //Serial.print("DEBUG OFF\n");
       int tmp = 930;
        mesh.write(&tmp, 'M', sizeof(tmp),99);
        
        }
    else if (input_string == "99 31") {
        debug = 1;  
        //Serial.print("DEBUG ON\n");
        int tmp = 931;
        mesh.write(&tmp, 'M', sizeof(tmp),99);
        
        } */   
    input_string = "";
    string_complete = false;
    }


