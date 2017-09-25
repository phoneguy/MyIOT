/*
    8 channel relay module
    

           g 1 2 3 4 5 6 7 8 v
           n                 c
           d                 c
*/



void update_relays()
{
    digitalWrite(RELAY1_PIN, relay1_state);
    digitalWrite(RELAY2_PIN, relay2_state);
}

