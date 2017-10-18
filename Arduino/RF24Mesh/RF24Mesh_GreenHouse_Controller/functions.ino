


static void node_command() {

   if(network.available()){
       
        RF24NetworkHeader header;
        network.peek(header);

    uint32_t dat    = 0;
    char data[32]   = "";
    uint8_t rx_command = 0;
    uint8_t rx_state = 0;
    uint8_t relay = 0;
    uint8_t relay_pin = 0;
    uint8_t state = 0;

    switch(header.type){
        // Display the incoming millis() values from the sensor nodes
        case 'M':
        if (blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 255, 0, 0);
            }
        network.read(header,&dat,sizeof(dat));
        if (debug == 1) {
            Serial.println(dat);
        }
        if ( dat >= 10 && dat <= 81 ) { // Command range 10 to 99
            rx_command = dat;
            relay = rx_command / 10;
            relay_pin = relays[relay][1];
            rx_state = (rx_command - (relay * 10));
            state = relays[relay][2];

            if (rx_state > 1 ) {
                rx_state = state;
            }
            else {
                relays[relay][2] = rx_state;
                digitalWrite(relay_pin, rx_state);
            }
        }
        else if (dat >= 82 && dat <= 87) {
            update_rate = update_table[dat][1] * 1000;   
            }
        else if (dat == 90) {
            debug = 0;   
            }
        else if (dat == 91) {
            debug = 1;  
            }   
        break;
      
        case 'S':
        if (blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 0, 255, 0);
            }
        network.read(header, &data, sizeof(data));
        Serial.print(data);
        data[32] = "";
        break;
      
        default:
        if (blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 0, 0, 255);
            }
        network.read(header,0,0);
        Serial.println(header.type);
        break;
    }
    
  }
}

static void serial_command() {

    while (Serial.available()) {
        char inChar = (char)Serial.read();
        input_string += inChar;
        if (inChar == '\n') {
        string_complete = true;
        }
    }

    uint8_t rx_command = 0;
    uint8_t rx_state = 0;
    uint8_t relay = 0;
    uint8_t relay_pin = 0;
    uint8_t state = 0;
    
    rx_command = input_string.toInt();

if (rx_command >= 10 && rx_command <= 81) {

    input_string = "";
    string_complete = false;
    relay = rx_command / 10;
    relay_pin = relays[relay][1];
    rx_state = (rx_command - (relay * 10));
    state = relays[relay][2];
  
    if ( rx_state >= 2) { 
        rx_state = state;
    }
    relays[relay][2] = rx_state; // update relay state
    digitalWrite(relay_pin, rx_state);  // toggle pin
    }
    else if (rx_command == 90) {
        debug = 0;
    }
    else if (rx_command == 91) {
        debug = 1;
    }
    else {
    input_string = "";
    string_complete = false;
    }

}


static void scan_i2cbus() {
    Serial.begin (9600);
    Serial.println ();
    Serial.println ("I2C bus scanner");
    byte count = 0;
 
    for (byte i = 1; i < 128; i++) {
        Wire.beginTransmission (i);
        if (Wire.endTransmission () == 0) {
            Serial.print ("Found address: ");
            if (i < 16) {
                Serial.print("0x0");
                Serial.print(i, HEX);
                Serial.print(", ");
                if (i == BLINKM_ADDRESS) {
                    Serial.print("blinkm");
                    }
                Serial.println(""); 
            }
            else {
            Serial.print("0x");
            Serial.print(i, HEX);
            Serial.print(", ");
                if (i == HMC5883_ADDRESS) {
                    Serial.print("hmc4883");
                    }
                else if (i == BMA180_ADDRESS) {
                    Serial.print("bma180");
                    }
                else if (i == ITG3200_ADDRESS) {
                    Serial.print("itg3200");
                    }
                else if (i == BMP085_ADDRESS) {
                    Serial.print("bmp085");
                    }
                else if (i == MPU6050_ADDRESS) {
                    Serial.print("mpu6050");
                    }
                else if (i == MS5611_ADDRESS) {
                    Serial.print("ms5611");
                    }
                else if (i == ADXL345_ADDRESS) {
                    Serial.print("adxl345");
                    }    
                                    
             Serial.println("");       
            }
        count++;
        delay (10);
        }
    }
     
    Serial.println("Done.");
    Serial.print("Found ");
    Serial.print(count, DEC);
    Serial.println(" device(s).");

}

