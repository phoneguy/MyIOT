
//
// cmd device      state
// --------------------------------
// 10  relay 1      off
// 11  relay 1      on
// 20  relay 2      off
// 21  relay 2      on
// 30  relay 3      off
// 31  relay 3      on
// 40  relay 4      off
// 41  relay 4      on
// 50  relay 5      off
// 51  relay 5      on
// 60  relay 6      off
// 61  relay 6      on
// 70  relay 7      off
// 71  relay 7      on
// 80  relay 8      off
// 81  relay 8      on
// 82  1 second     update rate
// 83  2 second     update rate
// 84  5 second     update rate
// 85  10 second    update rate
// 86  30 second    update rate
// 87  60 second    update rate
// 88
// 89
// 90  serial debug on
// 91  serial debug off
// 92  update rate  1 seconds
// 93  update rate  2 seconds
// 94  update rate  5 seconds
// 95  update rate 10 seconds
// 96  update rate 30 seconds
// 97  update rate 60 seconds
// 98
// 99
//


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

        case 'M':
        if (blinkm == 1 && blinkm_state == 1) {
            blinkm_setrgb(BLINKM_ADDRESS, 255, 0, 0);
            }
        network.read(header,&dat,sizeof(dat));
        if (debug == 1) {
            // mesh.write(&dat, 'M', sizeof(dat));
            Serial.println(dat);
        }
        if (dat >= 10 && dat <= 81) {
//                  if (dat >= min_relays && dat <= max_relays) {

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
        else if (dat >= 82 && dat <= 87) {
            update_rate = update_table[dat - 82][1] * 1000;   
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

