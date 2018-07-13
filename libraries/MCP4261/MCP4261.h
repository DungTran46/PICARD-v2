
// MCP4261 2-channel Digital Potentiometer
// ww1.microchip.com/downloads/en/DeviceDoc/22059b.pdf

#ifndef MCP4261_h
#define MCP4261_h
#include <Arduino.h>
#include <SPI.h>
//Memory addresses
#define VOLATILE_WIPER_0 0x00
#define VOLATILE_WIPER_1 0x01
#define NON_VOLATILE_WIPER_0 0x02
#define NON_VOLATILE_WIPER_1 0x03
#define VOLATILE_TCON_REG	0x04
#define STATUS_REG				0x05
#define DATA_EEPROM_0 			0x06
#define DATA_EEPROM_1 			0x07
#define DATA_EEPROM_2 			0x08
#define DATA_EEPROM_3 			0x09
#define DATA_EEPROM_5 			0x0A
#define DATA_EEPROM_6 			0x0B
#define DATA_EEPROM_7 			0x0C
#define DATA_EEPROM_8 			0x0D
#define DATA_EEPROM_9 			0x0E
#define DATA_EEPROM_10 			0x0F
#define RESOLUTION_8BITS		256
#define TYPICAL_RW				75
#define RAB						10000
class MCP4261
{
  public:
    // You must at least specify the slave select pin and the rated resistance
	
    MCP4261(uint8_t slave_select, uint8_t sck, uint8_t sdi, uint8_t sdo);
	
	void begin();
	
	float getRs();
	
	uint16_t set_wiper0(int rwa);
	
	void increment();
	void disableWP();
	
	

	
    // // Not implemented
    // // Connect / disconnect potentiometers
    // bool pot0_connected(bool terminal_a, bool wiper, bool terminal_b);
    // bool pot1_connected(bool terminal_a, bool wiper, bool terminal_b);
    // void pot0_connect(bool terminal_a, bool wiper, bool terminal_b);
    // void pot1_connect(bool terminal_a, bool wiper, bool terminal_b);
    // 
    // bool pot0_shutdown();
    // bool pot1_shutdown();
    // void pot0_shutdown(bool shutdown);
    // void pot1_shutdown(bool shutdown);
    // 
    // bool hw_shutdown();

  protected:
    // Other devices can be configured below vv as per the device numbering scheme:
    // MCP4N-- N=1 single pot, N=2 dual pot
    // MCP4--N N=1 potentiometer, N=2 rheostat
    // MCP4-N- N=3 7-bit volatile, N=4 7-bit non-volatile, N=5 8-bit volatile, N=6 8-bit non-volatile
    float rs;

    uint8_t slave_select_pin, sck_pin, sdi_pin, sdo_pin;
	
	const static uint8_t MASK_8			= 0x00;
	const static uint16_t MASK			= 0x0000;
    const static uint8_t CMD_READ       = 0b00001100;
    const static uint8_t CMD_WRITE      = 0b00000000;
	const static uint8_t CMD_INCREAMENT= 0b00000100;
	const static uint8_t CMD_DECREAMENT= 0b00001000;
    const static uint8_t kADR_VOLATILE     = 0b00000000;
    const static uint8_t kADR_NON_VOLATILE = 0b00100000;

    const static uint8_t kTCON_REGISTER    = 0b01000000;
    const static uint8_t kSTATUS_REGISTER  = 0b01010000;
	
	uint16_t get_wiper_pos(int rwa);
	uint16_t contruct_commandByte(uint8_t memory_address, uint8_t command_bits);
    uint16_t construct_dataByte(int rwa);
	uint16_t command(uint16_t commandByte, uint16_t dataByte);
	void write_16(uint8_t address, int rwa, bool isVolatile);
	uint8_t increment_command();
	uint8_t write_8(uint8_t commandByte);
	
	
	

    //uint16_t read(byte cmd_byte);
    //void write(byte cmd_byte, byte data_byte);
};

#endif // Mcp4261_h



