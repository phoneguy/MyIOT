/*
        HMC5883 Compass
*/

void hmc5883_init(void) {
    if ( compass == 1) {    
        // Initialize and set continuous mode

        int n = Wire.requestFrom(HMC5883_ADDRESS, 2);
        if (n == 2) // two bytes received
        { 
        Wire.beginTransmission(HMC5883_ADDRESS);
        Wire.write(0x02);    
        Wire.write(0x00);
        Wire.endTransmission();
        compass_state = 1;
        Serial.println("hmc5883 ok");
        } else {
        compass_state = 0;
        }
    
    }
     
}

static int read_compass() {
    Wire.beginTransmission(HMC5883_ADDRESS);
    Wire.write(0x03); // Select register 3, X MSB register
    Wire.endTransmission();
   
    // Read data from each axis, 2 registers per axis
    Wire.requestFrom(HMC5883_ADDRESS, 6);
    if(6<=Wire.available()) {
    x = Wire.read() << 8; // X msb
    x |= Wire.read();     // X lsb
    y = Wire.read() << 8; // Z msb
    y |= Wire.read();     // Z lsb
    z = Wire.read() << 8; // Y msb
    z |= Wire.read();     // Y lsb
    }
   
}


