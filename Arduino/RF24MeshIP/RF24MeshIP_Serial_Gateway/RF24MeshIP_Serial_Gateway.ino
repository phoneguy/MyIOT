 
 /*
  * RF24Mesh Sensor network serial gateway
  * stevenharsanyi@gmail.com
  * September 20, 2017 v1.0
  */
  
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>

/***** Configure the chosen CE,CS pins *****/
RF24 radio(9, 10); // Mega 2560
RF24Network network(radio);
RF24Mesh mesh(radio,network);

#define LED_TXRX          // Flash LED on SLIP device TX or RX 
#define SLIP_DEBUG        // Will delay and flash LEDs if unable to find a node by IP address ( node needs to reconnect via RF24Mesh ) 
#define DEBUG_LED_PIN A3

#define UIP_BUFFER_SIZE MAX_PAYLOAD_SIZE

uint8_t slip_buf[UIP_BUFFER_SIZE]; // MSS + TCP Header Length

uint32_t timer = 0;
uint8_t debug = 0;
boolean string_complete = false;
String input_string = "";
char packet_one[32] = "";


//Function to send incoming network data to the SLIP interface
void networkToSLIP();
 
void setup() {
  
    Serial.begin(9600);
   
    // Set the nodeID to 0 for the master node
    mesh.setNodeID(0);
    Serial.println("Hardware ID: RF24MESHGATEWAY");
    Serial.print("Node ID: ");
    Serial.println(mesh.getNodeID());
    
    // Connect to the mesh
    mesh.begin();
    //mesh.setStaticAddress(99, 02);
    //mesh.setStaticAddress(98, 03);
    
    // Use the serial port as the SLIP device
    slipdev_init(Serial);
    
// LED stuff
  pinMode(DEBUG_LED_PIN, OUTPUT);
#if defined (SLIP_DEBUG)
  digitalWrite(DEBUG_LED_PIN, HIGH);
  delay(200);
  digitalWrite(DEBUG_LED_PIN, LOW);
#endif

}
uint32_t active_timer =0;

void loop() {    
     // Provide RF24Network addresses to connecting & reconnecting nodes
  if(millis() > 10000){
    mesh.DHCP();
  }

  }
     // Check for incoming data from the sensors
    if(network.available()){
        RF24NetworkHeader header;
        network.peek(header);
    
        uint32_t dat = 0;
        char data[32] = "";
    
        switch(header.type){
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
    
        serial_event();
    }
    
    if (debug == 1) {
        if(millis() - timer > 30000){
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
if(mesh.update() == EXTERNAL_DATA_TYPE) {
    networkToSLIP();
  }

}

// Read from serial port and send node id and command
static void serial_event() {

   while (Serial.available()) {
        char rx_character = (char)Serial.read();
        input_string += rx_character;

        if (rx_character == '\n') {
        string_complete = true;
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
        }
        
    input_string = "";
    string_complete = false;

}



void networkToSLIP(){
  
    RF24NetworkFrame *frame = network.frag_ptr;
    size_t size = frame->message_size;
    uint8_t *pointer = frame->message_buffer;
    slipdev_send(pointer, size);
    //digitalWrite(DEBUG_LED_PIN, !digitalRead(DEBUG_LED_PIN));
    
}

void flashLED() {
#if defined (SLIP_DEBUG)
  digitalWrite(DEBUG_LED_PIN, HIGH);
  delay(200);
  digitalWrite(DEBUG_LED_PIN, LOW);
  delay(200);
  digitalWrite(DEBUG_LED_PIN, HIGH);
  delay(200);
  digitalWrite(DEBUG_LED_PIN, LOW);
  delay(200);
  digitalWrite(DEBUG_LED_PIN, HIGH);
  delay(200);
  digitalWrite(DEBUG_LED_PIN, LOW);
  delay(200);
#endif

}


