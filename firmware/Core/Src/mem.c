#include "mem.h"

/* USER CODE BEGIN 0 */
#define EEPROM_ADDRESS 0xA0

#define PAGE_SIZE 64
#define PAGE_NUM 250


uint16_t bytestowrite(uint16_t size, uint16_t offset){
  return size;
}

void mem_write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{

	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(PAGE_SIZE)/log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos = 0;

	// write the data
	for (int i = 0; i < numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
	    uint16_t bytesremaining = bytestowrite(size, offset);  // calculate the remaining bytes to be written

		HAL_StatusTypeDef return_code = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, MemAddress, 2, &data[pos], bytesremaining, 1000);  // write the data to the EEPROM

		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay (5);  // Write cycle delay (5ms)
	}
}

void mem_read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	int paddrposition = log(PAGE_SIZE)/log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos = 0;

	for (int i = 0; i < numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		HAL_StatusTypeDef return_code = HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, MemAddress, 2, &data[pos], bytesremaining, 1000);
		startPage += 1;
		offset = 0;
		size = size-bytesremaining;
		pos += bytesremaining;
	}
}

void mem_write_uint8(uint8_t number, uint16_t page, uint16_t offset){
    uint8_t data_to_write[1] = {electrical_angle_offset};
    eeprom_write(page, offset, data_to_write, 1);
}

uint8_t mem_read_uint8(uint16_t page, uint16_t offset){
    uint8_t data_to_read[1];
    eeprom_read(page, offset, data_to_read, 1);

    return data_to_read[0];
}

void write_encoder_params(){
    uint8_t data_to_write[1] = {electrical_angle_offset};

    mem_write_uint8(electrical_angle_offset,
                    PARAM_ENCODER_OFFSET_PAGE,
                    PARAM_ENCODER_OFFSET_BYTE);
}

void read_encoder_params() {
    electrical_angle_offset = mem_read_uint8(
                    PARAM_ENCODER_OFFSET_PAGE,
                    PARAM_ENCODER_OFFSET_BYTE);
}