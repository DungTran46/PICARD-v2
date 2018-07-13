#include "MCP4261.h"
MCP4261 pot = MCP4261(21,5,18,19);
int count = 0;
void setup() {
  // put your setup code here, to run once:
  delay(3000);
  Serial.begin(9600);
  pot.begin();
  //pot.disableWP();  
}

void loop() {
    if (count<30){
      pot.increment();
      delay(100);
      count = count+1;
    }
  
  
  // put your main code here, to run repeatedly:
  //pot.increment();
  //delay(500);
}
