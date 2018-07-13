#include "MCP4261.h"
MCP4261 mcp = MCP4261(21,5,18,19);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(mcp.getRs());
  mcp.begin();
  delay(5000);
  uint16_t temp=mcp.set_wiper0(1000);
  Serial.print("Result: "); Serial.println(temp);

}

void loop() {
}
