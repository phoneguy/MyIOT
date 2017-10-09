/*
 * BlinkMCommunicator -- Communication gateway between a computer and a BlinkM
 *                       Essentially turns an Arduino to an I2C<->serial adapter
 *
 * Command format is:
 * pos description
 *  0   <startbyte>
 *  1   <i2c_addr>
 *  2   <num_bytes_to_send>
 *  3   <num_bytes_to_receive>
 *  4   <send_byte0>
 *  5..n [<send_byte1>...]
 *
 * Thus minimum command length is 5 bytes long, for reading back a color, e.g.:
 *   {0x01,0x09,0x01,0x01, 'g'}
 * Most commands will be 8 bytes long, say to fade to an RGB color, e.g.:
 *   {0x01,0x09,0x04,0x00, 'f',0xff,0xcc,0x33}
 * The longest command is to write a script line, at 12 bytes long, e.g.:
 *   {0x01,0x09,0x08,0x00, 'W',0x00,0x01,50,'f',0xff,0xcc,0x33}
 * 
 * BlinkM connections to Arduino
 * -----------------------------
 * PWR - -- gnd -- black -- Gnd
 * PWR + -- +5V -- red   -- 5V
 * I2C d -- SDA -- green -- Analog In 4
 * I2C c -- SCK -- blue  -- Analog In 5
 * 
 *
 * 2007-11, Tod E. Kurt, ThingM, http://thingm.com/
 *
 */

void blinkm_init()
{
  if (blinkm == 1) {
    Wire.beginTransmission(BLINKM_ADDRESS);
    Wire.write('a');
    Wire.endTransmission();

    Wire.requestFrom(BLINKM_ADDRESS, 1);
    if( Wire.available() ) {
    byte n=Wire.read();
        if (n == BLINKM_ADDRESS) {
            blinkm_off(BLINKM_ADDRESS);            
            Serial.println("blinkm ok");
            blinkm_state = 1;
            } else 
            blinkm_state = 0;
     }
  }
}     

void blinkm_setrgb(byte address, byte red, byte green, byte blue) 
{
    Wire.beginTransmission(address);
    Wire.write('n');
    Wire.write(red);
    Wire.write(green);
    Wire.write(blue);
    Wire.endTransmission();
  
}

void blinkm_fadergb(byte address, byte red, byte green, byte blue) 
{
    Wire.beginTransmission(address);
    Wire.write('c');
    Wire.write(red);
    Wire.write(green);
    Wire.write(blue);
    Wire.endTransmission();
  
}

void blinkm_off(byte address) 
{
    Wire.beginTransmission(address);
    Wire.write('o');
    Wire.endTransmission();
    blinkm_setrgb(BLINKM_ADDRESS, 0, 0, 0);

}

