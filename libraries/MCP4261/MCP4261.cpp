
// MCP4261 2-channel Digital Potentiometer
// ww1.microchip.com/downloads/en/DeviceDoc/22059b.pdf

// The default spi Control Register - SPCR = B01010000;
// interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
// sample on leading edge of clk,system clock/4 rate (fastest).
// Enable the digital pins 11-13 for spi (the MOSI,MISO,spiCLK)
#include <spi.h>

#include "MCP4261.h"
SPIClass spi(HSPI);

//---------- constructor ----------------------------------------------------
MCP4261::MCP4261(uint8_t slave_select, uint8_t sck, uint8_t sdi, uint8_t sdo)
{
	//setup slave select pin
	slave_select_pin=slave_select;
	//setup other pins
	sck_pin=sck;
	sdi_pin=sdi;
	sdo_pin=sdo;
	
	//calculate rs
	rs=RAB/RESOLUTION_8BITS;
}

//------------------ protected -----------------------------------------------

//get wiper postion with a desired Rwa
uint16_t MCP4261::get_wiper_pos(int rwa)
{
	uint16_t num_rs;
	if(rwa<TYPICAL_RW)
		num_rs=TYPICAL_RW; //smallest resistance 
	else if (rwa<RAB)
		num_rs=(rwa-TYPICAL_RW)/rs;
	else
		num_rs=0;	// largeest resistance RAB
	return num_rs;
}
/*
	Function constructs commandByte
	memory_address: desired memory address from 0x00 to 0x0F
	command_bits: define type of operation 
		0b00: write
		0b11: read
	refer to page 45 of MCP4261 datasheet if need more info
*/
uint16_t MCP4261::contruct_commandByte(uint8_t memory_address, uint8_t command_bits)
{
	uint16_t temp=0x0000;
	temp=(temp|(memory_address << 4)|command_bits)<<8;
	return temp;
}

/*
	contruct increment commmand
*/
uint8_t MCP4261::increment_command()
{
	return ((VOLATILE_WIPER_0) << 4)|CMD_INCREAMENT|MASK_8;
}

uint16_t MCP4261:: construct_dataByte(int rwa)
{
	return get_wiper_pos(rwa);
}

/* contruct 16bit data with desired rwa*/
uint16_t MCP4261:: command(uint16_t commandByte, uint16_t dataByte )
{
	uint16_t temp= commandByte|MASK;
	return (commandByte|MASK)|(dataByte|MASK);
}
/*write 16 bytes*/
void MCP4261::write_16(uint8_t address, int rwa, bool isVolatile)
{
	Serial.println();
	uint16_t commandByte=contruct_commandByte(address,CMD_WRITE);
	uint16_t dataByte=construct_dataByte(rwa);
	uint16_t package= command(commandByte, dataByte );
	Serial.println(package);
	digitalWrite(slave_select_pin,LOW);
	spi.write16(package);
	if(!isVolatile)
		delay(10);
	digitalWrite(slave_select_pin,HIGH);
}

uint8_t MCP4261::write_8(uint8_t commandByte)
{
	uint8_t recv_package=0;
	digitalWrite(slave_select_pin,LOW);
	spi.write(commandByte);
	digitalWrite(slave_select_pin,HIGH);
	return recv_package;
}

//---------- public ----------------------------------------------------
void MCP4261:: begin()
{
	spi.begin(sck_pin, sdo_pin, sdi_pin, slave_select_pin);
	digitalWrite(slave_select_pin,HIGH);
	spi.setClockDivider(SPI_CLOCK_DIV32);
	spi.setBitOrder(MSBFIRST);
	spi.setDataMode(SPI_MODE2);
	spi.setHwCs(true);
}
float MCP4261:: getRs()
{
	return rs;
}
/*set wiper0 to specific location*/
uint16_t MCP4261::set_wiper0(int rwa)
{
  write_16(NON_VOLATILE_WIPER_0,rwa,true);
  return 0;
}

/*send increment the wiper0*/
void MCP4261::increment(){
	write_8(increment_command());
}

void MCP4261::disableWP()
{
	uint8_t temp= 0b11110100;
	write_8(temp);
}
