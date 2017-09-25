


static void node_command() {

   if(network.available()){
       
        RF24NetworkHeader header;
        network.peek(header);

    uint32_t dat=0;
    char data[32]="";
    
    switch(header.type){
        // Display the incoming millis() values from the sensor nodes
        case 'M':
        if (blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 255, 0, 0);
            }
        network.read(header,&dat,sizeof(dat));
        if (debug == 1) {
           // mesh.write(&dat, 'M', sizeof(dat));
            Serial.println(dat);
        }
        if(dat == 10) {
            digitalWrite(RELAY1_PIN, RELAY_OFF);
            relay1_state = 0;
            }
        else if (dat == 11) {
            digitalWrite(RELAY1_PIN, RELAY_ON);
            relay1_state = 1;
            }
        else if (dat == 20) {
            digitalWrite(RELAY2_PIN, RELAY_OFF);
            relay2_state = 0;
            }
        else if (dat == 21) {
            digitalWrite(RELAY2_PIN, RELAY_ON);
            relay2_state = 1;
            }
        else if (dat == 30) {
            debug = 0;   
            }
        else if (dat == 31) {
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

