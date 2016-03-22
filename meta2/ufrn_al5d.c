/*************************************************************/
/*       UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE         */
/*   DEPARTAMENTO DE ENGENHARIA DE COMPUTAÇÃO E AUTOMAÇÃO    */
/*							     */
/*   DRIVER DO BRAÇO ROBÓTICO LYNX AL5D PARA A PORTA SERIAL  */
/*							     */
/*   DESENVOLVEDORES:					     */
/*	- ENG. M.SC. DESNES AUGUSTO NUNES DO ROSÁRIO	     */
/*	- ENG. DANILO CHAVES DE SOUSA ICHIHARA		     */
/*************************************************************/

#include "ufrn_al5d.h"

//Initialize serial port
int configurar_porta(int fd)
{
	int portstatus = 0;

	struct termios options;
	// Get the current options for the port...
	tcgetattr(fd, &options);
	// Set the baud rates to 115200...
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD);

	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	//options.c_cflag |= SerialDataBitsInterp(8);           /* CS8 - Selects 8 data bits */
	options.c_cflag &= ~CRTSCTS;                            // disable hardware flow control
	options.c_iflag &= ~(IXON | IXOFF | IXANY);           // disable XON XOFF (for transmit and receive)
	//options.c_cflag |= CRTSCTS;                     /* enable hardware flow control */

	options.c_cc[VMIN] = 0;     //min carachters to be read
	options.c_cc[VTIME] = 0;    //Time to wait for data (tenths of seconds)

	//Set the new options for the port...
	tcflush(fd, TCIFLUSH);

	if (tcsetattr(fd, TCSANOW, &options)==-1)
	{
		perror("On tcsetattr:");
		portstatus = -1;
	}
	else
		portstatus = 1;

	return portstatus;
}


int abrir_porta(void)
{
	int fd; // file description for the serial port
	
	fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	
	if(fd == -1) // if open is unsucessful
	{
		//perror("open_port: Unable to open /dev/ttyS0 - ");
		perror("open_port: Unable to open /dev/ttyS0. \n");
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
		//fcntl(fd, F_SETFL, FNDELAY);
		//printf("port is open.\n");
	}
	
	return(fd);
} //open_port

int enviar_comando(char* data, int fd)
{
	int n;

	//printf("%s[%d]\n", data, strlen(data));

	data[strlen(data)] = 0x0d;
	data[strlen(data)+1] = 0x00;

	n = write(fd, data, strlen(data));

	if (n < 0)
	{
		fputs("write() falhou!\n", stderr);
		return -1;
	}
	else
	{
		//data[strlen(data)] = '\0';
		//printf("%s[%d]\n", data, strlen(data));
		//printf("Successfully wrote %d bytes\n",n);
		return n;
	}
}

void fechar_porta(int fd)
{
	close(fd);
}

unsigned int trava(unsigned int canal, unsigned int pos)
{
	switch(canal)
	{
		case BAS_SERVO:
			if(pos<BAS_MIN)
				return BAS_MIN;
			else if (pos > BAS_MAX)
				return BAS_MAX;
			else
				return pos;
		break;
		case SHL_SERVO:
			if(pos<SHL_MIN)
				return SHL_MIN;
			else if (pos > SHL_MAX)
				return SHL_MAX;
			else
				return pos;
		break;
		case ELB_SERVO:
			if(pos<ELB_MIN)
				return ELB_MIN;
			else if (pos > ELB_MAX)
				return ELB_MAX;
			else
				return pos;
		break;
		case WRI_SERVO:
			if(pos<WRI_MIN)
				return WRI_MIN;
			else if (pos > WRI_MAX)
				return WRI_MAX;
			else
				return pos;
		break;
		case GRI_SERVO:
			if(pos<GRI_MIN)
				return GRI_MIN;
			else if (pos > GRI_MAX)
				return GRI_MAX;
			else
				return pos;
		break;
		default:
			if(pos<500)
				return 500;
			else if (pos > 2500)
				return 2500;
			else
				return pos;
		break;
	}
}

void ufrn_header(void)
{
	printf("/*************************************************************/\n");
	printf("/*       UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE         */\n");
	printf("/*   DEPARTAMENTO DE ENGENHARIA DE COMPUTAÇÃO E AUTOMAÇÃO    */\n");
	printf("/*							     */\n");
	printf("/*        DRIVER DO BRAÇO ROBÓTICO LYNX AL5D - v2.0 	     */\n");
	printf("/*							     */\n");
	printf("/*   DESENVOLVEDORES:					     */\n");
	printf("/*	- ENG. M.SC. DESNES AUGUSTO NUNES DO ROSÁRIO	     */\n");
	printf("/*	- ENG. DANILO CHAVES DE SOUSA ICHIHARA		     */\n");
	printf("/*************************************************************/\n\n");
}

