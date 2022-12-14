//***Machine states and error codes***
#define ERROR       	-1
#define	RECV_OVERFLOW	-10
#define NO_CHAR_RECEIVED    -15
#define	BAD_START_DELIMITER	-20
#define	BAD_FRAME_TYPE	-30
#define	BAD_DELIVERY_STATUS	-40
#define IDLE        	1
#define RECEIVING   	10
#define	SENDING			30
#define	CHECK_DATA		50
#define	DONE			1000
#define DEBUG       	999

//***UART settings. Use UART1***
#define UART_BUFFER_DIM 64 //MAX Dimension of received data from Xbee
#define UART_ID uart1
#define BAUD_RATE 19200
#define UART_TX_PIN 8 //GP8
#define UART_RX_PIN 9 //GP9

//***i2c Settings***
#define ADDR _u(0x76) // device has default bus address of 0x76
// hardware registers of BMP280
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)

#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)

#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

// calibration registers
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)

#define NUM_CALIB_PARAMS 24 // number of calibration registers to be read
