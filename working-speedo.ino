#include <avr/IO.h>
#include <stdio.h>
#include <math.h>

#define DEBOUCE_COUNT 1000

int WHEEL_DI = 1; //In m
int FASTEST_POSSIBLE = 75;
float cirum = PI * WHEEL_DI;

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
unsigned int prev_state;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  DDRD = 0b11000000; // Take pD2 as sensor; pD3 as reset; pD6 and pD7 A and B on 7 seg
  DDRB = 0b11111111; // B as output; C to G to dp on 7 seg
  DDRC = 0b11111111; // C as output; distance d1 - d4
  //              speed d3 - d4

  PORTD = 0b00110111; //Set to all normally high, would get pulled to ground by the hall effect?
  //The output gets set to 0
  PORTB = 0b00000000; //All output to 0
  PORTC = 0b00000000; //All output to 0
  prev_state = 0b11111111;
  previousTime = 0;
}

void loop() {
  // put your main code here, to run repeatedly:

  pin_read = PIND;
  if (((pin_read | 0b11110111) == 0b11111111) && (distance != 0)) { // Reset button has been clicked
    distance = 0;
    Serial.println("Reset button");
    //displayDist();
  }
  
  if ((pin_read | 0b11111011) == 0b11111111) { //If there is one reading that is high restart debounce count 
    count = 0;
  }

  if (((pin_read | 0b11111011) == 0b11111011) && (count < DEBOUCE_COUNT)) { //Count 100 debounce then start calculations 
    count++;

    if (count == DEBOUCE_COUNT) {
      currentTime = millis();
      cycleTime = currentTime - previousTime;

      Serial.print("cycleTime is ");
      Serial.println(cycleTime);
      if(cycleTime > 80){ //If this is a legitimate reading
         velocity = (cirum* 60 * 60) / (cycleTime ); //Velocity in long form, check formula
        dis_velocity = (int) velocity;
  
  
        distance += cirum;
        dis_distance = int(distance/10);
        /*Alternate method:
          count++;
          distance = count*WHEEL_DI;*/

        previousTime = currentTime;
  
        Serial.print("velocity is ");
        Serial.println(dis_velocity);
        Serial.print("distance is ");
        Serial.println(dis_distance);
      }
    }
    //Put update displays here 
  }
}
