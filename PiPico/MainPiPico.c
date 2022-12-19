/**
 * Xbee PRO S1 configured as API 1
 * Baud rate = 19200 b/s
 * 
 */
#include "Globals.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"


const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint GP16_PIN = 16; //pin 21
const uint GP28_PIN = 28; //pin 34

struct funct_out {
	bool	Done;
	bool	Error;
	int		Status;
	int		ErrorID;
};

struct structSystemTimer {
	bool g_50ms_tick;
	bool g_100ms_tick;
	bool g_200ms_tick;
	bool g_500ms_tick;
	bool g_30s_tick;
	bool g_1s_tick;
	bool g_os_100ms;
	bool g_os_1s;
	bool g_os_1m;
	absolute_time_t g_now;
} g_systemTimers;

void SYSTEM_Clock() {
static bool firstScan=0, pr_50ms=false, pr_500ms=false, pr_30s=false;
static unsigned char	_500ms_Ctr=0, _1s_ctr=0;
static 	absolute_time_t endTime, now;

	if (!firstScan) {
		firstScan = true;
		endTime = make_timeout_time_ms(50); //50ms timeout
	}

	now = get_absolute_time(); //Actual time
	g_systemTimers.g_now = now;

	if (absolute_time_diff_us(endTime, now) > 0) { //Check for timeout
		g_systemTimers.g_50ms_tick = !g_systemTimers.g_50ms_tick;
		endTime = make_timeout_time_ms(50); //50ms timeout
	}

	g_systemTimers.g_os_100ms = g_systemTimers.g_50ms_tick & !pr_50ms;
	pr_50ms = g_systemTimers.g_50ms_tick;
	
	if (g_systemTimers.g_os_100ms)
		if ( (++_500ms_Ctr % 5) == 0) {
			g_systemTimers.g_500ms_tick = !g_systemTimers.g_500ms_tick;
			_500ms_Ctr = 0;
		}
	g_systemTimers.g_os_1s = g_systemTimers.g_500ms_tick & !pr_500ms;
	pr_500ms = g_systemTimers.g_500ms_tick;
	
	if (g_systemTimers.g_os_1s)
		if ( (++_1s_ctr % 30) == 0) {
			g_systemTimers.g_30s_tick = ! g_systemTimers.g_30s_tick;
			_1s_ctr = 0;
		}		

	g_systemTimers.g_os_1m = g_systemTimers.g_30s_tick & !pr_30s;
	pr_30s = g_systemTimers.g_30s_tick;
}

unsigned char calc_CRC(char *inData, int nrOfBytes) {
	unsigned char CRC;
	unsigned int sum;
	int i;
	
	sum = 0;
	for(i=0; i < nrOfBytes; i++)
		sum += *inData++;
	
	CRC = 0xFF - (sum & 0xFF);
	return CRC;
}

struct funct_out sendData(bool inExecute, int inDeltaCtr, int inTemperature, int inPressure) {
	static bool execute_pr;
	//b0 - Frame: stast delimiter 0x7E, b1-b2 Message length, b3 - Frame type
	//b4 - Frame ID, b5 - b6 Dest.address, b7 - Options, b8-b9 pulse counter,
	//b9-b10 temperature[°C*10],  ..N Data
	//EOF (checksum)
	static char sendMsg[] = {0x7E,0x00,0x0B,0x01,0x01,0x00,0x01,0x00,0x3A,0x00,0x00,0x00,0x00,0x00,0xC2}; //Fixed length
	static int status=IDLE, dataIdx;
    static absolute_time_t endTime;
	static struct funct_out outStruct;
	bool execute_os;
	int recvMsgLength;

	char recvData[UART_BUFFER_DIM], ch, *firstDataPtr;
	int i, msgLength;

    absolute_time_t now;

	execute_os = inExecute & !execute_pr;
	execute_pr = inExecute;

	outStruct.Status = status;

	switch(status) {
		case IDLE:
			outStruct.Done = false;
			outStruct.Error = false;
			outStruct.ErrorID = 0;
			if(execute_os)
				status = SENDING;
			break;
			
		case SENDING:
			firstDataPtr = &sendMsg[8]; //Start of data area
			*firstDataPtr = inDeltaCtr >> 8; //byte 8 - Counter MSB
			*(++firstDataPtr) = inDeltaCtr & 0xFF; //byte 9
			*(++firstDataPtr) = inTemperature >> 8; //byte 10 - temperature MSB
			*(++firstDataPtr) = inTemperature & 0xFF; //byte 11
			*(++firstDataPtr) = inPressure >> 8; //byte 12 - pressure MSB
			*(++firstDataPtr) = inPressure & 0xFF; //byte 13
			msgLength = sizeof(sendMsg);
			sendMsg[msgLength-1] = calc_CRC(&sendMsg[3], msgLength - 4); //Checksum
			for (i=0; i < sizeof(sendMsg); i++)
				uart_putc_raw(UART_ID, sendMsg[i]); //TBD: escape chars
			bzero(recvData, sizeof(recvData) ); //Reset received area
			endTime = make_timeout_time_ms(2000); //2s timeout
			dataIdx=0;
			status = RECEIVING;
			break;

		case RECEIVING:
			if(uart_is_readable(UART_ID)) { //Chek for data
				recvData[dataIdx++] = uart_getc(UART_ID);
				if (dataIdx == (UART_BUFFER_DIM)) { //Too many bytes received
					outStruct.ErrorID = RECV_OVERFLOW;
					status = ERROR;
				}				
			}

            now = get_absolute_time(); //Actual time
			if (absolute_time_diff_us(endTime, now) > 0)  //Check for timeout
				status = CHECK_DATA;
			break;

		case CHECK_DATA: //TBD check data integrity
			if (dataIdx == 0) {
				outStruct.ErrorID = NO_CHAR_RECEIVED;
				printf("[DEBUG] No char received.\n");
				status = ERROR;
				break;
			}

			if (recvData[0] != 0x7E) { //Start delimiter
				printf("[DEBUG] Bad start delimiter.\n");
				outStruct.ErrorID = BAD_START_DELIMITER;
				status = ERROR;
				break;
			}

			recvMsgLength = recvData[1] | recvData[2]; //Check byte order and message length

			if (recvData[3] != 0x89) { //Frame type
				printf("[DEBUG] Bad frame type.\n");
				outStruct.ErrorID = BAD_FRAME_TYPE;
				status = ERROR;
				break;
			}
			if (recvData[5] != 0) {//Delivery status 0=OK, 1=NO ACK
				outStruct.ErrorID = BAD_DELIVERY_STATUS;
				printf("[DEBUG] Bad delivery status.\n");
				status = ERROR;
				break;
			}
			printf("[DEBUG] ACK Ok.\n");

			//TBD: checksum chek
			status = DONE; //If check is Ok
			break;
		
		case ERROR:
			outStruct.Error = true;
			if (!inExecute)
				status = IDLE;
			break;
			
		case DONE:
			outStruct.Done = true;
			status = IDLE;
			break;
	}

	return outStruct;
}

struct bmp280_calib_param {
	   // temperature params
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    // pressure params
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
};

#ifdef i2c_default
int bmp280_init() {
	int retVal;
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];

    // 500ms sampling time, x16 filter
    const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;
	
    // send register number followed by its corresponding value
    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;
    retVal = i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
	
	if (retVal == PICO_ERROR_GENERIC)
		return -1;
    
	// osrs_t x1, osrs_p x4, normal mode operation
    const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    retVal = i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
	
	if (retVal == PICO_ERROR_GENERIC )
		return -2;

	return 0;
}

void bmp280_read_raw(int32_t* temp, int32_t* pressure) {
    // BMP280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
    // note: normal mode does not require further ctrl_meas and config register writes

    uint8_t buf[6];
    uint8_t reg = REG_PRESSURE_MSB;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(i2c_default, ADDR, buf, 6, false);  // false - finished with bus

    // store the 20 bit read in a 32 bit signed integer for conversion
    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
}

void bmp280_reset() {
    // reset the device with the power-on-reset procedure
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
}

// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
int32_t bmp280_convert(int32_t temp, struct bmp280_calib_param* params) {
    // use the 32-bit fixed point compensation implementation given in the
    // datasheet
    
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
    return var1 + var2;
}

int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
    int32_t t_fine = bmp280_convert(temp, params);
    return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp, struct bmp280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = bmp280_convert(temp, params);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}

void bmp280_get_calib_params(struct bmp280_calib_param* params) {
    // raw temp and pressure values need to be calibrated according to
    // parameters generated during the manufacturing of the sensor
    // there are 3 temperature params, and 9 pressure params, each with a LSB
    // and MSB register, so we read from 24 registers

    uint8_t buf[NUM_CALIB_PARAMS] = { 0 };
    uint8_t reg = REG_DIG_T1_LSB;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    // read in one go as register addresses auto-increment
    i2c_read_blocking(i2c_default, ADDR, buf, NUM_CALIB_PARAMS, false);  // false, we're done reading

    // store these in a struct for later use
    params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];
}

#endif

int SYSTEM_Init() {
	int retVal;

	stdio_init_all();
	gpio_init(GP16_PIN); //pin 21, counter input
    gpio_set_dir(GP16_PIN, GPIO_IN);

	gpio_init(GP28_PIN); //pin 34, LED
    gpio_set_dir(GP28_PIN, GPIO_OUT);

	gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);


// I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C); //GP4, pin6
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C); //GP5, pin7
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // configure BMP280
	if ( (retVal = bmp280_init()) != 0) {
		gpio_put(LED_PIN, true);
		printf("bmp280_init error. Returned value: %d.\n", retVal);
		return retVal;
		}
	// retrieve fixed compensation params
    struct bmp280_calib_param params;
    bmp280_get_calib_params(&params);

	printf("Init OK.\n");
	return retVal;
}


bool R_TRIG(bool clk, bool *prValue) {
	bool return_os;
	return_os = clk & !*prValue;
	*prValue = clk;
	return return_os;
}

int main() {
	bool execute=false, ctr_pin, pr_Ctr=false, gp16_pin_os, gp16_pin_pr=false;
	int status=IDLE, i, deltaCtr, retVal;
	int32_t raw_temperature, raw_pressure, temperature, pressure;
	long int	pulseCtr=0, oldPulseCtr=0;
	struct funct_out retStruct;
	struct bmp280_calib_param params;
	absolute_time_t ledStop_t;

	if ( (retVal= SYSTEM_Init()) != 0)
		return retVal; //In case of error abort program

	bmp280_get_calib_params(&params); //retrieve fixed compensation params. TBD: error handling

	while(true) {
		SYSTEM_Clock();
		
		gp16_pin_os = R_TRIG(gpio_get(GP16_PIN), &gp16_pin_pr);
		if (gp16_pin_os)
			++pulseCtr;

		if (g_systemTimers.g_os_1m) {
			bmp280_read_raw(&raw_temperature, &raw_pressure);
        	temperature = bmp280_convert_temp(raw_temperature, &params);
        	pressure = bmp280_convert_pressure(raw_pressure, raw_temperature, &params);
        	printf("Pressure = %.3f kPa\tTemp. = %.2f C\n", pressure / 100.f, temperature / 100.f); //Debug

			deltaCtr = pulseCtr - oldPulseCtr;
			execute = true;
		}
		
		retStruct=sendData(execute, deltaCtr, (int16_t) temperature, (int16_t) (pressure/10.f) );
		
		gpio_put(LED_PIN, execute); //Debug

		if (retStruct.Done) {
			gpio_put(GP28_PIN, true);
			ledStop_t = make_timeout_time_ms(3000); //3s timeout
			execute=false;
			oldPulseCtr += deltaCtr; //Avoid pulseCtr increment during sending
		}

		//Keep LED on for 3s
		if (absolute_time_diff_us(ledStop_t, g_systemTimers.g_now) > 0)
			gpio_put(GP28_PIN, false);

		if (retStruct.Error)
			execute=false;
	}
}

