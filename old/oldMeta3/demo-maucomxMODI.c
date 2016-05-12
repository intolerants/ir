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
#define HOME_POS "#0P1640#1P1468#2P1672#3P1500#4P1500"
#define PI 3.14159265
#define px 0
#define py 0

int main()
{
	ufrn_header();

	int serial_fd;
	char *comando;

	// INICIO DO PROGRAMA DEMO //

	printf("PROGRAMA DEMONSTRACAO INICIADO\n\n");

	serial_fd = abrir_porta();

	if (serial_fd == -1)
	{
		printf("Erro abrindo a porta serial /dev/ttyS0\nAbortando o programa...");
		return -1;
	}
	else
	{
		printf("Porta serial /dev/ttyS0 aberta com sucesso\n");

		if (configurar_porta(serial_fd) == -1)
		{
			printf("Erro inicializando a porta\n");
			close(serial_fd);
			return -1;
		}

		comando = (char*) malloc(sizeof(char) * BUFSIZE);

		//////////////////////
		// PRIMEIRO COMANDO //
		//////////////////////
		printf("\nPRIMEIRO COMANDO - POSICAL INICIAL\n");

		sprintf(comando, "%s", HOME_POS);

		//Escrevendo com teste de escrita
		if (enviar_comando(comando, serial_fd) != -1)
		{
			printf("Enviando de comando com teste de envio: %s\n", HOME_POS);
		}
		else
		{
			printf("Problema no envio do comando\nAbortando o programa...");
			return -1;
		}

		printf("Pressione enter para continuar...");
		getchar();

		/////////////////////
		// SEGUNDO COMANDO //
		/////////////////////

		memset(comando, 0, BUFSIZE);

		char c = 'a';

		float X, Y, Z;

		unsigned int pos = 1500;
		float angSenseBas = 90;
		float angSenseShl = 90;
		float angSenseElb = 90;
		float angSenseWri = 90;
		float SenseBasAux = 1500;
		float SenseShlAux = 1500;
		float SenseElbAux = 1500;
		float SenseWriAux = 1500;
		float SenseGriAux = 1500;
		float SenseBas = 1640;
		float SenseShl = 1468;
		float SenseElb = 1672;
		float SenseWri = 1500;
		float SenseGri = 1500;

		int h = 7, L1 = 15, L2 = 19, L3 = 7, t1, t2, t3, t4;


		system("rm input.txt");
		system("cat /dev/null > input.txt");

		do {

			system ("clear");

			printf("Digite space para sair\n");

			t1 = angSenseBas, t2 = angSenseShl, t3 = angSenseElb, t4 = angSenseWri;

			X = -((cos((t1 * PI) / 180)) * (cos(((t2 + t3 + t4) * PI) / 180) * L3 + cos(((t2 + t3) * PI) / 180) * L2 + cos(((t2) * PI) / 180) * L1));

			Y = sin(((t1) * PI) / 180) * (cos(((t2 + t3 + t4) * PI) / 180) * L3 + cos(((t2 + t3) * PI) / 180) * L2 + cos(((t2) * PI) / 180) * L1);

			Z = (sin(((t2 + t3 + t4) * PI) / 180) * L3 + sin(((t2 + t3) * PI) / 180) * L2 + sin(((t2) * PI) / 180) * L1 + h);

			printf("A coordenada x do ponto : %.2f \n", X);
			printf("A coordenada y do ponto : %.2f \n", Y);
			printf("A coordenada z do ponto : %.2f \n", Z);

			printf("BASE     -> (Q)ESQUERDA ; (A)DIREITA  | LARGURA DO PULSO: %d \t | ANGULO: %d \n", (int)SenseBasAux, (int)angSenseBas);
			printf("OMBRO    -> (W)CIMA     ; (S)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %d \n", (int)SenseShlAux, (int)angSenseShl);
			printf("COTOVELO -> (E)CIMA     ; (D)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %d \n", (int)SenseElbAux, (int)angSenseElb);
			printf("PUNHO    -> (R)CIMA     ; (F)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %d \n", (int)SenseWriAux, (int)angSenseWri);
			printf("GARRA    -> (T)FECHAR   ; (G)ABRIR    | LARGURA DO PULSO: %d \t              \n", (int)SenseGriAux);
			printf("Digite space para sair\n");
			system("/bin/stty raw");
			c = getchar();
			system("/bin/stty cooked");


			if (c == 'q') {
				SenseBas = SenseBas + 4;
				SenseBasAux = SenseBasAux + 4;
				pos = SenseBas;
				angSenseBas = (SenseBasAux - 500) * 0.09;
				sprintf(comando, "#%dP%d", BAS_SERVO, trava(BAS_SERVO, pos));
			} else if (c == 'a') {
				SenseBas = SenseBas - 4;
				SenseBasAux = SenseBasAux - 4;
				pos = SenseBas;
				angSenseBas = (SenseBasAux - 500) * 0.09;
				sprintf(comando, "#%dP%d", BAS_SERVO, trava(BAS_SERVO, pos));

			} else if (c == 'w') {
				SenseShl = SenseShl + 4;
				SenseShlAux = SenseShlAux - 4;
				pos = SenseShl;
				angSenseShl = (SenseShlAux - 500) * 0.09;
				sprintf(comando, "#%dP%d", SHL_SERVO, pos * (pos < 1850) + 1850 * (pos >= 1850));
			} else if (c == 's') {
				SenseShl = SenseShl - 4;
				SenseShlAux = SenseShlAux + 4;
				pos = SenseShl;
				angSenseShl = (SenseShlAux - 500) * 0.09;
				sprintf(comando, "#%dP%d", SHL_SERVO, pos * (pos < 1850) + 1850 * (pos >= 1850));
			} else if (c == 'e') {
				SenseElb = SenseElb - 4;
				SenseElbAux = SenseElbAux - 4;
				pos = SenseElb;
				angSenseElb = (SenseElbAux - 500) * 0.09;
				sprintf(comando, "#%dP%d", ELB_SERVO, pos);
			} else if (c == 'd') {
				SenseElb = SenseElb + 4;
				SenseElbAux = SenseElbAux + 4;
				pos = SenseElb;
				angSenseElb = (SenseElbAux - 500) * 0.09;
				sprintf(comando, "#%dP%d", ELB_SERVO, pos);
			} else if (c == 'r') {
				SenseWri = SenseWri + 4;
				SenseWriAux = SenseWriAux + 4;
				pos = SenseWri;
				angSenseWri = (SenseWriAux - 500) * 0.09;
				sprintf(comando, "#%dP%d", WRI_SERVO, trava(WRI_SERVO, pos));
			} else if (c == 'f') {
				SenseWri = SenseWri - 4;
				SenseWriAux = SenseWriAux - 4;
				pos = SenseWri;
				angSenseWri = (SenseWriAux - 500) * 0.09;
				sprintf(comando, "#%dP%d", WRI_SERVO, trava(WRI_SERVO, pos));
			} else if (c == 't') {
				SenseGri = SenseGri + 10;
				SenseGriAux = SenseGriAux + 10;
				pos = SenseGri;
				sprintf(comando, "#%dP%d", GRI_SERVO, trava(GRI_SERVO, pos));
			} else if (c == 'g') {
				SenseGri = SenseGri - 10;
				SenseGriAux = SenseGriAux - 10;
				pos = SenseGri;
				sprintf(comando, "#%dP%d", GRI_SERVO, trava(GRI_SERVO, pos));
			} else if (c == 'p') {
				system("./meta3");
			} else if (c == '=') {
				FILE *f = fopen("input.txt", "w");
				if (f == NULL)
				{
					printf("Error opening file!\n");
					exit(1);
				}

				/* print some text */
				fprintf(f, "%d %d\n", X, Y);
				fclose(f);
			}


			if (enviar_comando(comando, serial_fd) != -1)
			{
				printf("\nEnviando de comando com teste\n");
			}
			else
			{
				printf("Problema no envio do comando\nAbortando o programa...");
				return -1;
			}

			memset(comando, 0, BUFSIZE);

		} while ( c != ' ');

		// FIM DO PROGRAMA DEMO //
		fechar_porta(serial_fd);
		printf("\nAcesso a porta serial /dev/ttyS0 finalizado\n");

	}

	printf("\nPROGRAMA FINALIZADO\n\n");

	return 0;
}
