/*************************************************************/
/*       UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE         */
/*   DEPARTAMENTO DE ENGENHARIA DE COMPUTAÇÃO E AUTOMAÇÃO    */
/*                               */
/*   DRIVER DO BRAÇO ROBÓTICO LYNX AL5D PARA A PORTA SERIAL  */
/*                               */
/*   DESENVOLVEDORES:                        */
/*  - ENG. M.SC. DESNES AUGUSTO NUNES DO ROSÁRIO         */
/*  - ENG. DANILO CHAVES DE SOUSA ICHIHARA           */
/*************************************************************/
 
#include <stdio.h>
#include <stdlib.h>
 
#include "ufrn_al5d.h"
 
//Posicao inicial para todos os servos
#define HOME_POS "#0P1528S500#1P1468S500#2P1672S500#3P1504S500#4P1870S500T2000"
#define STANDBY "#0P1528S500#1P1428S500#2P968S500#3P1504S500#4P1870S500T2000"
#define WAKEUP "#1P1428S500#2P968S500#3P1504S500#4P1870S500T2000"
#define RELAX "#0P0000S2000#1P0000S2000#2P0000S2000#3P0000S2000#4P0000S1000"
#define BACK "#0P1528S2000#1P1600S2000#2P1768S2000#3P1324S2000#4P1870S2000"
#define POS1 "#0P1532S1000#1P1088S1000#2P1612S1000#3P844S1000#4P1990S1000"
#define POS2 "#0P1288S1000#1P1344S1000#2P2060S1000#3P1008S1000#4P1990S1000"
#define POS3 "#0P1832S1000#1P1344S1000#2P2060S1000#3P1008S1000#4P1990S1000"
#define POS4 "#0P1532S1000#1P1088S1000#2P1612S1000#3P844S1000#4P1990S1000"
#define POS5 "#0P1640S1000#1P1180S1000#2P1808S1000#3P836S1000#4P1990S1000"
#define POS6 "#0P1476S1000#1P1180S1000#2P1808S1000#3P836S1000#4P1990S1000"
#define PI 3.14159265
#define px 0
#define py 0
#define OFFSETX 100
#define OFFSETY 200
#define offsetBas 664
#define offsetShl 680
#define offsetElb 2428
#define offsetWri 588
#define gainBas 0.1041667
#define gainShl 0.1142132
#define gainElb 0.1209677
#define gainWri 0.0982533
        // HOME_POS
        // sense[0] = 1528;
        // sense[1] = 1468;
        // sense[2] = 1672;
        // sense[3] = 1504;
        // sense[4] = 1870;
        // BACK
        // sense[0] = 1528;
        // sense[1] = 1796;
        // sense[2] = 2072;
        // sense[3] = 0844;
        // sense[4] = 1870;
        // RELAX
        // sense[0] = 0000;
        // sense[1] = 0000;
        // sense[2] = 0000;
        // sense[3] = 0000;
        // sense[4] = 0000;
     
float Senses[6][5] = {{1532, 1088, 1612, 844 , 1990},
                      {1288, 1344, 2060, 1008, 1990},
                      {1832, 1344, 2060, 1008, 1990},
                      {1532, 1088, 1612, 844 , 1990},
                      {1640, 1180, 1808, 836 , 1990},
                      {1476, 1180, 1808, 836 , 1990}};
 
 
int serial_fd;
char *comando, *last_comando;
 
int relax_flag = 0;
 
char c = 'a';
 
float X, Y, Z;
float x, y, z, phi = 0.0;
 
unsigned int pos = 1500;
float angSenseBas = -90;
float angSenseShl = 90;
float angSenseElb = -90;
float angSenseWri = 0;
float teta[4] = {0};
float sense[5] = {1528, 1468, 1672, 1504, 1870};
 
float L1 = 6.3, L2 = 14.6, L3 = 18.3, L4 = 8.5;
 
float h = 7, t1, t2, t3, t4;
int t = 2000, s = 300;
 
void send_command(void);
void make_and_send_command(void);
void rad2deg(float *ang);
void calc_tetas(float x, float y, float z, float phi);
void calc_senses(void);
void move(float x, float y, float z, float phi);
void pega(void);
void solta(void);
void repouso(void);
void pega_piloto(void);
void desenha(int *bolas, int n);
 
int main()
{
    ufrn_header();
 
     
 
    // INICIO DO PROGRAMA DEMO //
 
    printf("PROGRAMA INTOLERANTS INICIADO\n\n");
 
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
        last_comando = (char*) malloc(sizeof(char) * BUFSIZE);
 
        //////////////////////
        // PRIMEIRO COMANDO //
        //////////////////////
        printf("\nPRIMEIRO COMANDO - POSICAL INICIAL\n");
 
        sprintf(comando, "%s", STANDBY);
 
        //Escrevendo com teste de escrita
        if (enviar_comando(comando, serial_fd) != -1)
        {
            printf("Enviando de comando com teste de envio: %s\n", STANDBY);
        }
        else
        {
            printf("Problema no envio do comando\nAbortando o programa...");
            return -1;
        }
 
        printf("Pressione enter para continuar...");
        getchar();
        memset(comando, 0, BUFSIZE);
 
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
 
        memset(comando, 0, BUFSIZE);
 
        /////////////////////
        // SEGUNDO COMANDO //
        /////////////////////
 
 
 
        //system("rm input.txt");
        //system("cat /dev/null > input.txt");
        //system("rm pos.txt");
        //system("cat /dev/null > pos.txt");
 
        do {
 
            if(system ("clear"));
 
            t1 = angSenseBas*PI/180;
            t2 = angSenseShl*PI/180;
            t3 = angSenseElb*PI/180;
            t4 = angSenseWri*PI/180;

            X = cos(t1)*(L3*cos(t2 + t3) + L2*cos(t2) + L4*cos(t2 + t3 + t4));
 
            Y = sin(t1)*(L3*cos(t2 + t3) + L2*cos(t2) + L4*cos(t2 + t3 + t4));
 
            Z = L1 + L3*sin(t2 + t3) + L2*sin(t2) + L4*sin(t2 + t3 + t4);
            
            calc_tetas(X, Y, Z, phi);

            printf("A coordenada x do ponto : %.2f \n", X);
            printf("A coordenada y do ponto : %.2f \n", Y);
            printf("A coordenada z do ponto : %.2f \n", Z);
            printf("phi da ferramenta       : %.2f \n", angSenseShl+angSenseElb+angSenseWri);
            //printf("#0P%dS%d#1P%dS%d#2P%dS%d#3P%dS%d#4P%dS%dT%d\n", (int)sense[0], s,  (int)sense[1], s,  (int)sense[2], s,  (int)sense[3], s,  (int)sense[4], s, t);
            printf("%s\n", last_comando);
            printf("BASE     -> (Q)ESQUERDA ; (A)DIREITA  | LARGURA DO PULSO: %.2f \t | ANGULO: %.2f \t | ANGULOC: %.2f \n", sense[0], angSenseBas, teta[0]);
            printf("OMBRO    -> (W)CIMA     ; (S)BAIXO    | LARGURA DO PULSO: %.2f \t | ANGULO: %.2f \t | ANGULOC: %.2f \n", sense[1], angSenseShl, teta[1]);
            printf("COTOVELO -> (E)CIMA     ; (D)BAIXO    | LARGURA DO PULSO: %.2f \t | ANGULO: %.2f \t | ANGULOC: %.2f \n", sense[2], angSenseElb, teta[2]);
            printf("PUNHO    -> (R)CIMA     ; (F)BAIXO    | LARGURA DO PULSO: %.2f \t | ANGULO: %.2f \t | ANGULOC: %.2f \n", sense[3], angSenseWri, teta[3]);
            printf("GARRA    -> (T)FECHAR   ; (G)ABRIR    | LARGURA DO PULSO: %d \t              \n", (int)sense[4]);
            printf("Digite space para sair\n");
            if(system("/bin/stty raw"));
            c = getchar();
            if(system("/bin/stty cooked"));


            if (c == 'q') {
                sense[0] = sense[0] + 2;
                pos = sense[0];
                angSenseBas = (offsetBas - sense[0]) * gainBas;
                sprintf(comando, "#%dP%d", BAS_SERVO, trava(BAS_SERVO, pos));
            } else if (c == 'a') {
                sense[0] = sense[0] - 2;
                pos = sense[0];
                angSenseBas = (offsetBas - sense[0]) * gainBas;
                sprintf(comando, "#%dP%d", BAS_SERVO, trava(BAS_SERVO, pos));
            } else if (c == 'w') {
                sense[1] = sense[1] + 2;
                pos = sense[1];
                // angSenseShl = (sense[1] - offsetShl) * gainShl;
                angSenseShl = 762.684+7.948*sense[1];
                sprintf(comando, "#%dP%d", SHL_SERVO, pos * (pos < 1850) + 1850 * (pos >= 1850));
            } else if (c == 's') {
                sense[1] = sense[1] - 2;
                pos = sense[1];
                // angSenseShl = (sense[1] - offsetShl) * gainShl;
                angSenseShl = 762.684+7.948*sense[1];
                sprintf(comando, "#%dP%d", SHL_SERVO, pos * (pos < 1850) + 1850 * (pos >= 1850));
            } else if (c == 'e') {
                sense[2] = sense[2] - 2;
                pos = sense[2];
                // angSenseElb = (offsetElb - sense[2]) * gainElb - 180;
                angSenseElb = 854.377-9.231*sense[2];
                sprintf(comando, "#%dP%d", ELB_SERVO, pos);
            } else if (c == 'd') {
                sense[2] = sense[2] + 2;
                pos = sense[2];
                // angSenseElb = (offsetElb - sense[2]) * gainElb - 180;
                angSenseElb = 854.377-9.231*sense[2];
                sprintf(comando, "#%dP%d", ELB_SERVO, pos);
            } else if (c == 'r') {
                sense[3] = sense[3] + 2;
                pos = sense[3];
                // angSenseWri = (sense[3] - offsetWri) * gainWri - 90;
                angSenseWri = 1465.639+10.114*sense[3];
                sprintf(comando, "#%dP%d", WRI_SERVO, trava(WRI_SERVO, pos));
            } else if (c == 'f') {
                sense[3] = sense[3] - 2;
                pos = sense[3];
                // angSenseWri = (sense[3] - offsetWri) * gainWri - 90;
                angSenseWri = 1465.639+10.114*sense[3];
                sprintf(comando, "#%dP%d", WRI_SERVO, trava(WRI_SERVO, pos));
            } else if (c == 't') {
                sense[4] = sense[4] + 10;
                pos = sense[4];
                sprintf(comando, "#%dP%d", GRI_SERVO, trava(GRI_SERVO, pos));
            } else if (c == 'g') {
                sense[4] = sense[4] - 10;
                pos = sense[4];
                sprintf(comando, "#%dP%d", GRI_SERVO, trava(GRI_SERVO, pos));
            } else if (c == 'p') {
                if(system("gnome-terminal -x /home/aluno/intolerants/meta3/meta3"));
            } else if (c == '=') {
                FILE *f = fopen("input.txt", "a");
                if (f == NULL)
                {
                    printf("Error opening file!\n");
                    exit(1);
                }
 
                /* print some text */
                fprintf(f, "%d %d\n", (int)X+OFFSETX, (int)Y+OFFSETY);
                fclose(f);
                FILE *f2 = fopen("pos.txt", "a");
                if (f2 == NULL)
                {
                    printf("Error opening file!\n");
                    exit(1);
                }
 
                /* print some text */
                fprintf(f2, "#0P%uT1000#1P%uT1000#2P%uT1000#3P%uT1000#4P%uT1000\n", (unsigned int)sense[0], (unsigned int)sense[1], (unsigned int)sense[2], (unsigned int)sense[3], (unsigned int)sense[4]);
                 
                fclose(f2);
            } else if (c == 'b') {
            	int *bolas, i, n;
            	printf("\nDigite quantas bolinhas:\n");
            	scanf("%d", &n);
            	bolas = malloc(n*sizeof(int));
            	printf("\nDigite a ordem das bolinhas:\n");
            	for (i = 0; i < n; i++)
            		scanf("%d", &bolas[i]);
            		//bolas[i] = i+1;
            	desenha(bolas, n);
            } else if (c == '0') {
                // sense[0] = 1528;
                // sense[1] = 1796;
                // sense[2] = 2072;
                // sense[3] = 844;
                // sense[4] = 1870;
                // // send_command(1528,1796,2072,844,1870);
                // int delay = 2000;
                // sprintf(comando,"#0P%dT%d#1P%dT%d#2P%dT%d#3P%dT%d#4P%dT%d", sense[0], s,  sense[1], s,  sense[2], s,  sense[3], s,  sense[4], s, t);
                repouso();
            } else if (c == '1') {
            	move(2.97,-22.55,9,-5);
            	sleep(3);
            	move(6.02,-27.16,9.63,-10.38);
            	sleep(3);
            } else if (c == 'h') {
                printf("Insira s e t: ");
                if(scanf("%d", &s));
                if(scanf("%d", &t));
 
                sprintf(comando, "#0P1528S%d#1P1468S%d#2P1672S%d#3P1504S%d#4P1870S%dT%d", s, s, s, s, s, t);
            } else if (c == 'z') {
 				pega();
 				// getchar();
 			} else if (c == 'x') {
 				solta();
 				// printf("solta\n");
 				// getchar();
            } else if (c == 'm'){
                printf("\n\n");
                printf("x: ");
                if(scanf("%f", &x));
                printf("y: ");
                if(scanf("%f", &y));
                printf("z: ");
                if(scanf("%f", &z));
                printf("phi: ");
                if(scanf("%f", &phi));
                move(x,y,z,phi);
                printf("%s\n", comando);
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
 
            // while ( (c = getchar()) != '\n');
            memset(comando, 0, BUFSIZE);
 
        } while ( c != ' ');
        
        sprintf(comando, "%s", STANDBY);
        enviar_comando(comando, serial_fd);
        memset(comando, 0, BUFSIZE);
        printf("Pressione enter para relaxar...");
        getchar();
        sprintf(comando, "%s", RELAX);
        enviar_comando(comando, serial_fd);

        // FIM DO PROGRAMA DEMO //
        fechar_porta(serial_fd);
        printf("\nAcesso a porta serial /dev/ttyS0 finalizado\n");
 
    }
 
    printf("\nPROGRAMA FINALIZADO\n\n");
 
    return 0;
}
 
void send_command(void) {
    enviar_comando(comando,serial_fd);
    sprintf(last_comando, "%s", comando);
    memset(comando, 0, BUFSIZE);
}
 
void make_and_send_command(void){
    sprintf(comando, "#0P%uS100#1P%uS100#2P%uS100#3P%uS100\n", (unsigned int)sense[0], (unsigned int)sense[1], (unsigned int)sense[2], (unsigned int)sense[3]);
    send_command();
}
 
void rad2deg(float *ang){
    *ang *= 180/PI;
}zz
 
void calc_tetas(float x, float y, float z, float phi) {
    int i;
    // printf("\nCALC_TESTAS - x:%.2f y:%.2f z:%.2f phi%.2f\n", x, y, z, phi);
    phi *= PI/180;
    float exy = sqrt(pow(x,2) + pow(y,2));
    teta[0] = atan2(y/exy, x/exy);
    float x14 = exy - L4*cos(phi);
    float z14 = z - L1 - L4*sin(phi);
    float c3 = ((pow(x14, 2) + pow(z14, 2) - pow(L2, 2) - pow(L3, 2))/(2*L2*L3));
    float s3 = -sqrt(fabs(1-pow(c3,2)));
    // float s3 = -sqrt(1-pow(c3,2));
    teta[2] = atan2(s3,c3);
    float exz14 = sqrt(pow(x14,2) + pow(z14,2));
    float alpha = atan2(z14/exz14, x14/exz14);
    float beta = atan2(sin(teta[2])*L3/exz14, (L2 + L3*c3)/exz14);
    teta[1] = alpha - beta;
    teta[3] = phi - teta[1] - teta[2];
    // printf("\nexy:%.2f x14:%.2f s3sqrt:%.2f exz14:%.2f\n",exy,x14,1-pow(c3,2),exz14);
    for (i = 0; i < 4; i++){
        rad2deg(&teta[i]);
    }
}

void calc_senses(){
    /* Calibracao manual*/
    sense[0] = -(teta[0]/0.1041667 - 636 - 28);
    sense[1] = teta[1]/0.1142132 - 32 + 712;
    sense[2] = -((teta[2]+180)/0.1209677 - 2256 - 172);
    sense[3] = (teta[3]+90)/0.0982533 + 4 + 584;

    /* Calibracao por regressao linear */
    // sense[1] = 762.684+7.948*teta[1];
    // sense[2] = 854.377-9.231*teta[2];
    // sense[3] = 1465.639+10.114*teta[3];

    /* Calibracao quadrada*/
    // sense[1] = 0.0020*pow(teta[1],2) + 7.6037*teta[1] + 775.4128;
    // sense[2] = -0.0088*pow(teta[2],2) -10.9082*teta[2] + 783.4413;
    // sense[3] = 10.6*teta[3] + 1461.8;

    /* Calibracao de quarta ordem */
    // sense[1] = -9.16283572805788*pow(10,-06)*pow(teta[1], 4) + 0.00249238051918379*pow(teta[1], 3) - 0.220292028854092*pow(teta[1], 2) + 14.8408844789694*teta[1] + 712.940479709510;
    // sense[2] = 2.95031078343459*pow(10,-06)*pow(teta[2], 4) + 0.00133012241558227*pow(teta[2], 3) + 0.204563648896187*pow(teta[2], 2) + 3.35185281794406*teta[2] + 1113.04750020549;
    // sense[3] = -0.0485371397804810*pow(teta[3], 4) + 3.17374947124019*pow(teta[3], 3) - 74.7891701648607*pow(teta[3], 2) + 758.694940993243*teta[3] - 1195.19947031368;

}

void move(float x, float y, float z, float phi){
    printf("\nMOVE - x:%.2f y:%.2f z:%.2f phi%.2f\n", x, y, z, phi);

    calc_tetas(x, y, z, phi);
    // int i;
    // for (i = 0; i < 4; i++){
    //     printf("teta[%d]: %.2f ", i+1, teta[i]);
    // }
    // printf("\n");
    calc_senses();
    make_and_send_command();
    X = x;
    Y = y;
    Z = z;
    angSenseBas = (offsetBas - sense[0]) * gainBas;
    angSenseShl = (sense[1] - offsetShl) * gainShl;
    angSenseElb = (offsetElb - sense[2]) * gainElb - 180;
    angSenseWri = (sense[3] - offsetWri) * gainWri - 90;

}

void pega(){
	sprintf(comando, "#4P1950");
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
}

void solta(){
	sprintf(comando, "#4P1320");
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
}

void repouso(){
	sprintf(comando, "%s", STANDBY);
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
    printf("Pressione enter para relaxar...");
    getchar();
    sprintf(comando, "%s", RELAX);
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
    printf("Pressione enter para acordar...");
    getchar();
    sprintf(comando, "%s", WAKEUP);
    // sprintf(comando, "#1P1900S%d#2P2276S%d#3P1024S%d#4P1870S%dT%d", s, s, s, s, t);
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
}

void pega_piloto(){
	//antes do piloto
	move(0,-32.29,17.93,0);
	sleep(3);
	solta();
	sleep(2);
	//pega piloto
	//move(0,-36.94,15.97,-1.09);
	move(0,-37,15.88,-4.38);

	sleep(3);
	pega();
	sleep(2);
	//acima do piloto
	move(0,-34.72,24.36,11.51);
	sleep(3);
	//acima da area de trabalho
	move(0,-26.71,20.93,6.5);
	sleep(5);
}

void desenha(int *bolas, int n){
	int i;
	pega_piloto();
	for (i = 0; i < n; i++){
		switch(bolas[i]){
            case 1: 
            	move(2.97,-22.55,9.25,-12.96);
            	break;
            case 2:
            	move(6.02,-27.16,9.63,-10.38);
            	break;
            case 3:
            	move(2.94,-31.8,11.69,-3.07);
            	break;
            case 4:
            	move(-3.8,-31.6,11.69,-3.07);
            	break;
            case 5:
            	move(-6.2,-26.1,11,-3.07);
            	break;
            case 6:
            	move(-2.7,-20.3,9.1,-3.07);
            	break;
		}
		sleep(5);
	}
}