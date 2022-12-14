/* Running program on BBB
UART2 uses P9_21 and P9_22
*/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <error.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>

#define	APIKEY				"FF7ZI0TJZ1GY4TED"

#define	INIT				1
#define	IDLE				10
#define	RECEIVING			20
#define	ANALYZE_TELEGRAM	30
#define WRITE_TO_WEB		200
#define END_PROGRAM			900
#define	ERROR				-999

#define	RECV_BUFFER_LENGTH	128 //128 bytes
#define	RECV_MSG_LENGTH		11 //Received message length
#define	WEB_STORING_INTERVAL	300 //Storing interval on thingspeak.com

#define	OPEN_FILE_ERROR		-1
#define	NO_TTY_ERROR		-2
#define	TCGETATTR_ERROR		-3
#define	TCFLUSH_ERROR		-4
#define	TCSETATTR_ERROR		-5

struct struct_dataFromField {
	int energyPulses; //From SDM
	int	temperature; //From BME
	int pressure; //From BME
};
	
int stringTime(char *inTimeString) { //Converts actual time to string
	time_t now ;
    struct tm *tmpStruct_tm;

    time(&now);
	tmpStruct_tm = localtime(&now);
	strftime(inTimeString, 64, "%x - %H:%M", tmpStruct_tm);
return 0;
}
int SYSTEM_Init(char *device) {
	int fd;
	struct termios options;
	
	//Init UART

	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		perror("Failed to open the device.\n");
		return OPEN_FILE_ERROR;
	}
	
	if (!isatty(fd)) {
		printf("Error: No tty to open.\n");
		return NO_TTY_ERROR; 
	}
	
	if ( (tcgetattr(fd, &options)) < 0) {
		perror("Error during initialization. ");
		return TCGETATTR_ERROR;
	}
	options.c_cflag = B19200 | CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR | ICRNL;
	
	if ( (tcflush(fd, TCIFLUSH)) < 0) {
		perror("Error during initialization. ");
		return TCFLUSH_ERROR;
	}
	
	if ( (tcsetattr(fd, TCSANOW, &options)) < 0) {
		perror("Error during initialization. ");
		return TCSETATTR_ERROR;
	}
	
	return fd;
}

int ThingSpeakWrite(struct struct_dataFromField *inData, char *inApiKey) {
	int socketfd, portNumber, length, pulseCtr, temperature;
	char readBuffer[2000], message[255], Host[]="api.thingspeak.com";
	char data[32], head[512], number[8];
	struct sockaddr_in serverAddress;
	struct hostent *server;
	
	pulseCtr = inData->energyPulses;
	temperature = inData->temperature;
	printf("[D] Writing to TS: %d-%d.\n", pulseCtr, temperature);
	
	sprintf(message, "GET / HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", Host);
	
	//gethostbyname accepts a string name and returns host name struct
	server = gethostbyname(Host);
	if (server==NULL) {
		perror("Socket client: error - unable to resolv host name.\n");
		return -1;
		}
	//Create the socket of IP address type, SOCK_STREAM is for TCP
	socketfd = socket(AF_INET, SOCK_STREAM, 0);
	if (socketfd < 0) {
		perror("Socket Client: error opening TCP IP-based socket.\n");
		return -2;
		}
	//Clear the data in the serverAddress sockaddr_in struct
	bzero((char *) &serverAddress, sizeof(serverAddress));
	portNumber = 80;
	
	serverAddress.sin_family = AF_INET; //set the addr. family to be IP
	serverAddress.sin_port = htons(portNumber); //set port num to 80
	//Set addr to the resolved hostname addr
	bcopy((char *)server->h_addr, (char *)&serverAddress.sin_addr.s_addr, server->h_length);
	//Try to connect to server
	if (connect(socketfd, (struct sockaddr *) &serverAddress, sizeof(serverAddress)) < 0) {
		perror("Socket client: error connecting to the server.\n");
		return -3;
		}

	strcpy(data,"field1=");
	sprintf(number, "%d", pulseCtr);
	strcat(data, number);
	strcat(data,"&field2=");
	sprintf(number, "%.2f", temperature/100.f);
	strcat(data, number);
	strcat(data,"\n");
	
	strcpy(head, "POST /update HTTP/1.1\nHost: api.thingspeak.com\nConnection: close\nX-THINGSPEAKAPIKEY: ");
	strcat(head, inApiKey);
	strcat(head, "\nContent-Type: application/x-www-form-urlencoded\nContent-Length:");
	length = strlen(data);
	sprintf(number, "%d", length);
	strcat(head, number);
	strcat(head, "\n\n");
	
	strcat(head, data); //Rispetto all'esempio di Derek Molloy faccio una send sola, con due non andava.
	//Send the HTTP request string
	if (write(socketfd, head, sizeof(head)) < 0) {
		perror("Socket Client: error writing to socket.\n");
		return -4;
		}
	
	//Read  the HTTP response to a maximum of 2000 chars
	if (read(socketfd, readBuffer, sizeof(readBuffer)) < 0) {
		perror("Socket Client: error reading from socket.\n");
		return -5;
		}
	
	//printf("**START**\n%s\n**END**\n", readBuffer);
		
	close(socketfd);
	
	return 0;
}
	
int analyzeTelegram(char *inStringPtr, int inNrOfBytes, struct struct_dataFromField *inData) {
	char frameType, recvMsg[RECV_MSG_LENGTH], strTime[64];
	int msgLength, sourceAddr, rssi, options, i, crc;
	
	//Save data
	for (i=0; i < inNrOfBytes; i++)
		recvMsg[i] = *inStringPtr++;
		
	if (recvMsg[0] != 0x7E) {
		printf("Unknown start delimiter: 0x%X\n", recvMsg[0]);
		return -1;
		}
		
	msgLength = recvMsg[1] << 8 | recvMsg[2];
	if (msgLength != RECV_MSG_LENGTH) { //Fixed length 
		printf("Bad message length: %d\n", msgLength);
		return -2;
		}
	
	frameType = recvMsg[3];
	if (frameType != 0x81) {
		printf("Bad frame type: %X\n", frameType);
		return -3;
		}
	
	sourceAddr = recvMsg[4] << 8 | recvMsg[5];
	
	rssi = recvMsg[6];
	
	options = recvMsg[7];
	
	//Data masking
	inData->energyPulses = recvMsg[8] << 8 | recvMsg[9];
	inData->temperature = recvMsg[10] << 8 | recvMsg[11];
	inData->pressure = recvMsg[12] << 8 | recvMsg[13];
	
	//1 byte CRC, TBD check CRC
	crc = recvMsg[14];
	
	stringTime(strTime);
	printf("[D] %s\tRecv %d, RSSI:%d, pulses %d.\n", strTime, inNrOfBytes, rssi, inData->energyPulses);
	
	return 0;
}

int main() {
	char uartStr[]="/dev/ttyO2"; //UART4
	char recvString[RECV_BUFFER_LENGTH];
	char c, tick_50ms=0, pr_tick_50ms=0, run=1, sendToThingSpeak;
	char os_100ms, os_500ms;
	short	ctr_100ms=0;
	int uart, count, status=INIT, charCtr, receivingTimer, retVal, i;
	int pulseCtr, samplesCtr;
	long int delta_us, temperatureSum, pressureSum;
	double diff_t;
	time_t t_sendStart, t_Actual;
	struct timespec actualTime, previousTime;
	struct struct_dataFromField dataFromField, dataToWeb;
	
while(run) {
	clock_gettime(CLOCK_MONOTONIC_RAW, &actualTime);
	switch(status) {
		case INIT:
			printf("Program start.\n");
			if ( (uart = SYSTEM_Init(uartStr)) < 0) { 
				printf("Init failed! Return code: %d.\n", uart);
				status = ERROR;
				break;
			}
			clock_gettime(CLOCK_MONOTONIC_RAW, &previousTime);
			time(&t_sendStart);
			charCtr = 0;
			pulseCtr = 0;
			temperatureSum = 0;
			pressureSum = 0;
			samplesCtr = 0;
			sendToThingSpeak = 0;
			status = IDLE;
			break;
		
		case IDLE:
			if (sendToThingSpeak) {
				sendToThingSpeak = 0;
				time(&t_sendStart);
				status = WRITE_TO_WEB;
				break;
				}
				
			if (os_500ms) {
				//printf("[DEBUG] Leggo dalla seriale.\n");
				if ( (count = read(uart, &c,1)) < 0) {//Read from uart
					perror("Read error in IDLE.");
					status = END_PROGRAM;
					break;
				}
				if (count == 1)  {
					charCtr = 0;
					recvString[charCtr++] = c;
					receivingTimer = 0;
					i=0;
					status = RECEIVING;
					break;
				}	
			}

			break;
		
		case RECEIVING:
			if ( (count = read(uart, &c,1)) < 0) {//Read from uart
				perror("Read error in RECEIVING.");
				status = END_PROGRAM;
				break;
			}
			
			if (count == 1) {//One char ready
				recvString[charCtr++] = c;
				//TBD: check array limits
			}	
			
			if (os_100ms)
				if(receivingTimer++ == 5)  //About 500ms
					status = ANALYZE_TELEGRAM;
				
			break;
			
		case ANALYZE_TELEGRAM:
			if ( (retVal = analyzeTelegram(recvString, charCtr, &dataFromField)) < 0)
				status = ERROR;
			else { //Received telegram OK
				pulseCtr += dataFromField.energyPulses;
				temperatureSum += dataFromField.temperature;
				pressureSum += dataFromField.pressure;
				samplesCtr++;
				status = IDLE;
				}
			break;
		
		case WRITE_TO_WEB:
			dataToWeb.energyPulses = pulseCtr;
			
			if (samplesCtr) { //Check if at least one sample was stored
				dataToWeb.temperature = temperatureSum / samplesCtr; //Normally error is low, about 5/10000
				dataToWeb.pressure = pressureSum / samplesCtr;
				}
			else {
				dataToWeb.temperature = 300; //Send an impossible value
				dataToWeb.pressure = 10000; //Send an impossible value
				}
				
			if ( (retVal=ThingSpeakWrite(&dataToWeb, APIKEY)) == 0) {
				pulseCtr = 0; //Reset counter only when data is saved
				temperatureSum = 0;
				pressureSum = 0;
				samplesCtr = 0;
				}
			status = IDLE;
			break;
		
		case END_PROGRAM:
			close(uart);
			run=0;
			break;
		
		case ERROR:
			return -1;
			break;
	}
	time(&t_Actual);
	diff_t = difftime(t_Actual, t_sendStart);
	sendToThingSpeak = (diff_t >= WEB_STORING_INTERVAL); //5min
	
	delta_us = (actualTime.tv_sec - previousTime.tv_sec) * 1000000 + (actualTime.tv_nsec - previousTime.tv_nsec) / 1000;
	if (delta_us >= 50000) {//50ms
		clock_gettime(CLOCK_MONOTONIC_RAW, &previousTime);
		tick_50ms = !tick_50ms;
		}

	os_100ms = tick_50ms & !pr_tick_50ms;
	pr_tick_50ms = tick_50ms;
	if (os_100ms)
		ctr_100ms++;

	os_500ms = (ctr_100ms == 5);

	if (ctr_100ms == 5)
		ctr_100ms = 0;
	
}	
	printf("Program end.\n");
	
return 0;
}
