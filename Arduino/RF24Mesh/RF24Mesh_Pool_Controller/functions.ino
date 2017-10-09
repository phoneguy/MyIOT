


static void node_command() {

   if(network.available()){
       
        RF24NetworkHeader header;
        network.peek(header);

    uint32_t dat=0;
    char data[32]="";
    uint8_t rx_command = 0;
    uint8_t rx_state = 0;
    uint8_t relay = 0;
    uint8_t relay_pin = 0;
    uint8_t state = 0;
    
    switch(header.type){
        // Display the incoming millis() values from the sensor nodes
        case 'M':
        if (blinkm == 1 && blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 255, 0, 0);
            }
        network.read(header,&dat,sizeof(dat));
        if (debug == 1) {
            // mesh.write(&dat, 'M', sizeof(dat));
            Serial.println(dat);
        }
        if (dat >= min_relays && dat <= max_relays) {
            rx_command = dat;
            relay = rx_command / 10;
            relay_pin = relays[relay][1];
            rx_state = (rx_command - (relay * 10));
            state = relays[relay][2];

            if (rx_state > 1) {
                rx_state = state;
            }
            else {
                relays[relay][2] = rx_state;  // update array value
            }
            
            if (rx_state == 0) {
                digitalWrite(relay_pin, RELAY_OFF);
            }
            else if (rx_state == 1) {
                digitalWrite(relay_pin, RELAY_ON);
            }
        } 
        else if (dat == 90) {
            debug = 0;   
            }
        else if (dat == 91) {
            debug = 1;  
            }   
        break;
      
        case 'S':
        if (blinkm == 1 && blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 255, 255, 0);
            }
        network.read(header, &data, sizeof(data));
        Serial.println(data);
        data[32] = "";
        break;
      
        default:
        if (blinkm == 1 && blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 0, 0, 255);
            }
        network.read(header,0,0);
        Serial.println(header.type);
        break;
    }
    
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
                  blinkm_state = 1;
                    Serial.print("blinkm");
                    }
                Serial.println(""); 
            }
            else {
            Serial.print("0x");
            Serial.print(i, HEX);
            Serial.print(", ");
                if (i == HMC5883_ADDRESS) {
                    compass_state = 1;
                    Serial.print("hmc4883");
                    }
                else if (i == BMA180_ADDRESS) {
                    Serial.print("bma180");
                    }
                else if (i == ITG3200_ADDRESS) {
                    Serial.print("itg3200");
                    }
                else if (i == BMP085_ADDRESS) {
                  baro_state = 1;
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

