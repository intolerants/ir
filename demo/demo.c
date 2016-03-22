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

#include <stdio.h>
#include <stdlib.h>

#include "ufrn_al5d.h"

//Posicao inicial para todos os servos
#define HOME_POS "#0P1500#1P1500#2P1500#3P1500#4P1500"

int main()
{
	ufrn_header();

	int serial_fd;
	char *comando;
	
	serial_fd = abrir_porta();

	if(serial_fd == -1)
	{
		printf("Erro abrindo a porta serial /dev/ttyS0\nAbortando o programa...");
		return -1;
	}
	else
	{
		printf("PROGRAMA DEMONSTRACAO INICIADO\n\n");
		
		printf("Porta serial /dev/ttyS0 aberta com sucesso\n");

		if(configurar_porta(serial_fd) == -1)
		{
		    printf("Erro inicializando a porta\n");
		    close(serial_fd);
		    return -1;
		}

		comando = (char*) malloc(sizeof(char)*BUFSIZE);

		sprintf(comando,"%s",HOME_POS);

		//Escrevendo com teste de escrita
		if(enviar_comando(comando,serial_fd)!=-1)
		{
			printf("Enviando de comando com teste\n");		
		}
		else
		{
			printf("Problema no envio do comando\nAbortando o programa...");
			return -1;				
		}

		printf("Pressione enter para continuar...");
		getchar();

		printf("Espere 5 segundos...\n");
		sleep(5);

		// NECESSÁRIO: Limpar a regiao de memória da string //
		//             antes de mandar outro comando        //
		memset(comando, 0, BUFSIZE);

//		strcpy(comando,"#0P500");
//		sprintf(comando,"#%dP%d",SHL_SERVO,1000);
//		strcpy(comando,"#2P1500");
//		strcpy(comando,"#3P2000");
//		sprintf(comando,"#%dP%d",GRI_SERVO,1000);

		sprintf(comando,"#%dP%d",WRI_SERVO,500);

		printf("Envio de comando sem teste\n");
		enviar_comando(comando,serial_fd);

	        fechar_porta(serial_fd);
		printf("Acesso a porta serial /dev/ttyS0 finalizado\n");

		printf("\nPROGRAMA DEMONSTRACAO FINALIZADO\n\n");
    	}

	return 0;
}
