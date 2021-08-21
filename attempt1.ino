#include <avr/IO.h> 
#include <stdio.h>
#include <math.h>

int WHEEL_DI = 1; //In m
int FASTEST_POSSIBLE = 75;


unsigned int distance;
unsigned int dis_velocity;
unsigned int dis_distance;
int pin_read;
unsigned long previousTime;
unsigned long currentTime;
int firstSpin;
unsigned long cycleTime;
unsigned long velocity;
int count;

void setup() {
  // put your setup code here, to run once:
  DDRD = 0b11000000; // Take pD2 as sensor; pD3 as reset; pD6 and pD7 A and B on 7 seg
  DDRB = 0b11111111; // B as output; C to G to dp on 7 seg
  DDRC = 0b11111111; // C as output; distance d1 - d4
                     //              speed d3 - d4

  PORTD = 0b00111111; //Set to all normally high, would get pulled to ground by the hall effect?
                      //The output gets set to 0
  PORTB = 0b00000000; //All output to 0
  PORTC = 0b00000000; //All output to 0
}

void loop() {
  // put your main code here, to run repeatedly:
 
  pin_read = PIND;
  if((PIND | 0b111101111) == 0b11111111){
    distance = 0; 
    //displayDist();
  }
  if(((PIND | 0b11111011) == 0b11111011)&&(firstSpin == 0)){ // initialise previous time, could do this in setup otherwise?
    previousTime = millis();
    firstSpin++;
  }
  else if((PIND | 0b11111011) == 0b11111011){ // Assuming it is high and gets pulled down by hall effect 
    delay(70);
    currentTime = millis();
    cycleTime = currentTime - previousTime;
    if(cycleTime < 75){
      velocity = 0;
      dis_velocity = (int) velocity;
    }
    else{
      velocity = PI*WHEEL_DI/cycleTime*60*60; //Velocity in long form, check formula
      dis_velocity = (int) velocity;
    }
    
    distance += PI*WHEEL_DI;
    dis_distance = int(distance);
    /*Alternate method:
    count++;
    distance = count*WHEEL_DI;*/
    //CALL display setup and display loop from here 
    //displaysloop();
    previousTime = currentTime;
  }
  
  //if((PIND & 0b00000100) == 0b00000000) If it is pulled down not up
}
