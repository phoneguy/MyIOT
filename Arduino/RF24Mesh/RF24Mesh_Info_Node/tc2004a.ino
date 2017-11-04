/*
        TC2004A 20x4 LCD display
        
        The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
*/
/*
void start_screen(void) {
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("Pool Controller");
    lcd.setCursor(0, 1);
    lcd.print("Steven J. Harsanyi ");
    lcd.setCursor(0, 3);
    lcd.print("Version 1.0 2017");
    delay(2000);
}
void screen_a(void) {
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("Pool ");
    lcd.setCursor(5, 0);
    lcd.print(pool_temp);
    lcd.setCursor(8, 0);
    lcd.print("Air ");
    lcd.setCursor(12, 0);
    lcd.print(air_temp);
   
    lcd.setCursor(0, 1);
    lcd.print("Volts ");
    lcd.setCursor(6, 1);
    lcd.print(ac_voltage);
    lcd.setCursor(11, 1);
    lcd.print("Amps");
    lcd.setCursor(17, 1);
    lcd.print(ac_current);
    
    lcd.setCursor(0, 2);
    lcd.print("Humidity ");
    lcd.setCursor(9, 2);
    lcd.print(humidity);
    lcd.print("%");
    lcd.setCursor(14, 2);
    lcd.print("Mag ");
    lcd.setCursor(18, 2);
    lcd.print(z+x+y);
    
    lcd.setCursor(0, 3);
    lcd.print("Baro");
    lcd.setCursor(5, 3);
    lcd.print(air_pressure);
    lcd.setCursor(10, 3);
    lcd.print("Watts ");
    lcd.setCursor(16, 3);
    lcd.print(ac_watts);
}
/*
void screen_b(void) {
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("MagX ");
    lcd.setCursor(5, 0);
    lcd.print(x);
    lcd.setCursor(11, 0);
    lcd.print("MagY ");
    lcd.setCursor(16, 0);
    lcd.print(y);
    
    lcd.setCursor(0, 1);
    lcd.print("MagZ ");
    lcd.setCursor(6, 1);
    lcd.print(z);
        
    lcd.setCursor(0, 2);
    lcd.print("Relay 1 ");
    lcd.setCursor(9, 2);
    lcd.print(relay1_state);
    lcd.setCursor(11, 2);
    lcd.print("Relay 2 ");
    lcd.setCursor(17, 2);
    lcd.print(relay2_state);
    
    lcd.setCursor(0, 3);
    lcd.print("Watts ");
    lcd.setCursor(6, 3);
    lcd.print(ac_watts);
    
 }
*/
/*
void screen_c(void) {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("KwHr ");
    lcd.setCursor(5, 0);
    lcd.print(kilowatt_hour);
    lcd.setCursor(10, 0);
    lcd.print("Cost ");
    lcd.setCursor(15, 0);
    lcd.print(energy_cost);
    
    lcd.setCursor(0, 1);
    lcd.print("Hours ");
    lcd.setCursor(6, 1);
    lcd.print(hours);
    lcd.setCursor(10, 1);
    lcd.print("Mins ");
    lcd.setCursor(15, 1);
    lcd.print(minutes);
    
    lcd.setCursor(0, 2);
    lcd.print("Secs ");
    lcd.setCursor(9, 2);
    lcd.print(seconds);
    
    lcd.setCursor(0, 3);
    lcd.print("Real Power ");
    lcd.setCursor(11, 3);
    lcd.print(realPower);
    
}
*/


