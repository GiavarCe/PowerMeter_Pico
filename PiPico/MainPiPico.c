/**
 * GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> SDA on display
   board
   GPIO PICO_DEFAULT_I2C_SCK_PIN (on Pico this is GP5 (pin 7)) -> SCL on
   display board
   3.3v (pin 36) -> VCC on display board
   GND (pin 38)  -> GND on display board

 * Xbee PRO S1 configured as API 1
 * Baud rate = 19200 b/s
 * 
 */
#include "Globals.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "raspberry26x32.h"

// commands (see datasheet)
#define OLED_SET_CONTRAST _u(0x81)
#define OLED_SET_ENTIRE_ON _u(0xA4)
#define OLED_SET_NORM_INV _u(0xA6)
#define OLED_SET_DISP _u(0xAE)
#define OLED_SET_MEM_ADDR _u(0x20)
#define OLED_SET_COL_ADDR _u(0x21)
#define OLED_SET_PAGE_ADDR _u(0x22)
#define OLED_SET_DISP_START_LINE _u(0x40)
#define OLED_SET_SEG_REMAP _u(0xA0)
#define OLED_SET_MUX_RATIO _u(0xA8)
#define OLED_SET_COM_OUT_DIR _u(0xC0)
#define OLED_SET_DISP_OFFSET _u(0xD3)
#define OLED_SET_COM_PIN_CFG _u(0xDA)
#define OLED_SET_DISP_CLK_DIV _u(0xD5)
#define OLED_SET_PRECHARGE _u(0xD9)
#define OLED_SET_VCOM_DESEL _u(0xDB)
#define OLED_SET_CHARGE_PUMP _u(0x8D)
#define OLED_SET_HORIZ_SCROLL _u(0x26)
#define OLED_SET_SCROLL _u(0x2E)

#define OLED_ADDR _u(0x3C)
#define OLED_HEIGHT _u(32)
#define OLED_WIDTH _u(128)
#define OLED_PAGE_HEIGHT _u(8)
#define OLED_NUM_PAGES OLED_HEIGHT / OLED_PAGE_HEIGHT
#define OLED_BUF_LEN (OLED_NUM_PAGES * OLED_WIDTH)

#define OLED_WRITE_MODE _u(0xFE)
#define OLED_READ_MODE _u(0xFF)

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint GP16_PIN = 16; //pin 21
const uint GP28_PIN = 28; //pin 34

struct render_area {
    uint8_t start_col;
    uint8_t end_col;
    uint8_t start_page;
    uint8_t end_page;

    int buflen;
};

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

void fill(uint8_t buf[], uint8_t fill) {
    // fill entire buffer with the same byte
    for (int i = 0; i < OLED_BUF_LEN; i++) {
        buf[i] = fill;
    }
};

void fill_page(uint8_t *buf, uint8_t fill, uint8_t page) {
    // fill entire page with the same byte
    memset(buf + (page * OLED_WIDTH), fill, OLED_WIDTH);
};

// convenience methods for printing out a buffer to be rendered
// mostly useful for debugging images, patterns, etc

void print_buf_page(uint8_t buf[], uint8_t page) {
    // prints one page of a full length (128x4) buffer
    for (int j = 0; j < OLED_PAGE_HEIGHT; j++) {
        for (int k = 0; k < OLED_WIDTH; k++) {
            printf("%u", (buf[page * OLED_WIDTH + k] >> j) & 0x01);
        }
        printf("\n");
    }
}

void print_buf_pages(uint8_t buf[]) {
    // prints all pages of a full length buffer
    for (int i = 0; i < OLED_NUM_PAGES; i++) {
        printf("--page %d--\n", i);
        print_buf_page(buf, i);
    }
}

void print_buf_area(uint8_t *buf, struct render_area *area) {
    // print a render area of generic size
    int area_width = area->end_col - area->start_col + 1;
    int area_height = area->end_page - area->start_page + 1; // in pages, not pixels
    for (int i = 0; i < area_height; i++) {
        for (int j = 0; j < OLED_PAGE_HEIGHT; j++) {
            for (int k = 0; k < area_width; k++) {
                printf("%u", (buf[i * area_width + k] >> j) & 0x01);
            }
            printf("\n");
        }
    }
}

void calc_render_area_buflen(struct render_area *area) {
    // calculate how long the flattened buffer will be for a render area
    area->buflen = (area->end_col - area->start_col + 1) * (area->end_page - area->start_page + 1);
}

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

void oled_send_cmd(uint8_t cmd) {
    // I2C write process expects a control byte followed by data
    // this "data" can be a command or data to follow up a command

    // Co = 1, D/C = 0 => the driver expects a command
    uint8_t buf[2] = {0x80, cmd};
    i2c_write_blocking(i2c_default, (OLED_ADDR & OLED_WRITE_MODE), buf, 2, false);
}

void oled_send_buf(uint8_t buf[], int buflen) {
    // in horizontal addressing mode, the column address pointer auto-increments
    // and then wraps around to the next page, so we can send the entire frame
    // buffer in one gooooooo!

    // copy our frame buffer into a new buffer because we need to add the control byte
    // to the beginning

    // TODO find a more memory-efficient way to do this..
    // maybe break the data transfer into pages?
    uint8_t *temp_buf = malloc(buflen + 1);

    for (int i = 1; i < buflen + 1; i++) {
        temp_buf[i] = buf[i - 1];
    }
    // Co = 0, D/C = 1 => the driver expects data to be written to RAM
    temp_buf[0] = 0x40;
    i2c_write_blocking(i2c_default, (OLED_ADDR & OLED_WRITE_MODE), temp_buf, buflen + 1, false);

    free(temp_buf);
}

void oled_init() {
    // some of these commands are not strictly necessary as the reset
    // process defaults to some of these but they are shown here
    // to demonstrate what the initialization sequence looks like

    // some configuration values are recommended by the board manufacturer

    oled_send_cmd(OLED_SET_DISP | 0x00); // set display off

    /* memory mapping */
    oled_send_cmd(OLED_SET_MEM_ADDR); // set memory address mode
    oled_send_cmd(0x00); // horizontal addressing mode

    /* resolution and layout */
    oled_send_cmd(OLED_SET_DISP_START_LINE); // set display start line to 0

    oled_send_cmd(OLED_SET_SEG_REMAP | 0x01); // set segment re-map
    // column address 127 is mapped to SEG0

    oled_send_cmd(OLED_SET_MUX_RATIO); // set multiplex ratio
    oled_send_cmd(OLED_HEIGHT - 1); // our display is only 32 pixels high

    oled_send_cmd(OLED_SET_COM_OUT_DIR | 0x08); // set COM (common) output scan direction
    // scan from bottom up, COM[N-1] to COM0

    oled_send_cmd(OLED_SET_DISP_OFFSET); // set display offset
    oled_send_cmd(0x00); // no offset

    oled_send_cmd(OLED_SET_COM_PIN_CFG); // set COM (common) pins hardware configuration
    oled_send_cmd(0x02); // manufacturer magic number

    /* timing and driving scheme */
    oled_send_cmd(OLED_SET_DISP_CLK_DIV); // set display clock divide ratio
    oled_send_cmd(0x80); // div ratio of 1, standard freq

    oled_send_cmd(OLED_SET_PRECHARGE); // set pre-charge period
    oled_send_cmd(0xF1); // Vcc internally generated on our board

    oled_send_cmd(OLED_SET_VCOM_DESEL); // set VCOMH deselect level
    oled_send_cmd(0x30); // 0.83xVcc

    /* display */
    oled_send_cmd(OLED_SET_CONTRAST); // set contrast control
    oled_send_cmd(0xFF);

    oled_send_cmd(OLED_SET_ENTIRE_ON); // set entire display on to follow RAM content

    oled_send_cmd(OLED_SET_NORM_INV); // set normal (not inverted) display

    oled_send_cmd(OLED_SET_CHARGE_PUMP); // set charge pump
    oled_send_cmd(0x14); // Vcc internally generated on our board

    oled_send_cmd(OLED_SET_SCROLL | 0x00); // deactivate horizontal scrolling if set
    // this is necessary as memory writes will corrupt if scrolling was enabled

    oled_send_cmd(OLED_SET_DISP | 0x01); // turn display on
}

void render(uint8_t *buf, struct render_area *area) {
    // update a portion of the display with a render area
    oled_send_cmd(OLED_SET_COL_ADDR); //0x21 Set column address
    oled_send_cmd(area->start_col); //Start column
    oled_send_cmd(area->end_col); //End column

    oled_send_cmd(OLED_SET_PAGE_ADDR); //0x22 Set page address
    oled_send_cmd(area->start_page); //Start page
    oled_send_cmd(area->end_page); //End page

    oled_send_buf(buf, area->buflen);
}

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

//Sends an entire line (page) to SSD1306 display (font fixed to 8x8)
int ssd_1306_print(char *inString, int inPage) {
    int i, numCol, charTablePtr;
    char charToPrint;
    struct render_area area = {start_col: 0, end_col : OLED_WIDTH - 1, start_page : inPage, end_page : inPage};

    calc_render_area_buflen(&area);

    memset(stringToWrite, 0x00, sizeof(stringToWrite)/sizeof(stringToWrite[0])); //Clean string filling with spaces
    numCol = 0;

    while( ( (charToPrint = *inString++) != '\0') & (numCol < OLED_WIDTH) ) { //Scan string
//Calculate pointer to ASCII table, 32=SPACE is the first char, each char is represented by 8 bytes
        if ( (charToPrint < ' ') | (charToPrint > '~') ) //Check if char is in the table
            charToPrint = '#'; //Substitute char

        charTablePtr = (charToPrint -32) << 3;
        for (i=0; i<8; i++) //Move char representation (8 bytes long)
            stringToWrite[numCol++] = char_set_8x8[charTablePtr++];        
    }
    
    render(stringToWrite, &area);

    return 0;
}

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

   // run through the complete initialization process
    oled_init();
    // initialize render area for entire frame (128 pixels by 4 pages)
    //struct render_area frame_area = {start_col: 0, end_col : OLED_WIDTH - 1, start_page : 0, end_page : OLED_NUM_PAGES -1};
    struct render_area frame_area = {start_col: 0, end_col : OLED_WIDTH - 1, start_page : 0, end_page : OLED_NUM_PAGES - 1};
    calc_render_area_buflen(&frame_area);

    // zero the entire display
    uint8_t buf[OLED_BUF_LEN];
    fill(buf, 0x00);
    render(buf, &frame_area);

    // intro sequence: flash the screen 3 times
    for (int i = 0; i < 3; i++) {
        oled_send_cmd(0xA5); // ignore RAM, all pixels on
        sleep_ms(500);
        oled_send_cmd(0xA4); // go back to following RAM
        sleep_ms(500);
    }

    // configure horizontal scrolling
    oled_send_cmd(OLED_SET_HORIZ_SCROLL | 0x00);
    oled_send_cmd(0x00); // dummy byte
    oled_send_cmd(0x00); // start page 0
    oled_send_cmd(0x00); // time interval
    oled_send_cmd(0x03); // end page 3
    oled_send_cmd(0x00); // dummy byte
    oled_send_cmd(0xFF); // dummy byte

    // let's goooo!
    oled_send_cmd(OLED_SET_SCROLL | 0x01);

    // configure BMP280
	if ( (retVal = bmp280_init()) != 0) {
		gpio_put(LED_PIN, true);
		printf("bmp280_init error. Returned value: %d.\n", retVal);
		return retVal;
		}

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
    bool writeTemperatureToDisplay=false, writePressureToDsiplay=false;
	char displayString[17]; //TODO use parameter
    int status=IDLE, i, deltaCtr, retVal;
	int32_t raw_temperature, raw_pressure, temperature, pressure;
	long int	pulseCtr=0, oldPulseCtr=0;
	struct funct_out retStruct;
	struct bmp280_calib_param params;
	absolute_time_t ledStop_t;

	if ( (retVal= SYSTEM_Init()) != 0)
		return retVal; //In case of error abort program

	bmp280_get_calib_params(&params); //retrieve fixed compensation params. TBD: error handling

    ssd_1306_print("Init ok...", 0);

	while(true) {
		SYSTEM_Clock();
		
		gp16_pin_os = R_TRIG(gpio_get(GP16_PIN), &gp16_pin_pr);
		if (gp16_pin_os)
			++pulseCtr;
        
        //To riduce cycle overload write to display at different times
        if (writePressureToDsiplay) {
            writePressureToDsiplay = false;
            sprintf(displayString, "Press.=%.1f", pressure / 100.f);
            ssd_1306_print(displayString, 1);
            }

        if (writeTemperatureToDisplay) {
            writeTemperatureToDisplay = false;
            writePressureToDsiplay = true;
            sprintf(displayString, "Temp.=%.1f C", temperature / 100.f);
            ssd_1306_print(displayString, 0);
            }
            

		if (g_systemTimers.g_os_1m) {
			bmp280_read_raw(&raw_temperature, &raw_pressure);
        	temperature = bmp280_convert_temp(raw_temperature, &params);
        	pressure = bmp280_convert_pressure(raw_pressure, raw_temperature, &params);
        	printf("Pressure = %.3f kPa\tTemp. = %.2f C\n", pressure / 100.f, temperature / 100.f); //Debug

            writeTemperatureToDisplay = true;

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

