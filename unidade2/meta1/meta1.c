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
        // HOME_POS
        // SenseBas = 1528;
        // SenseShl = 1468;
        // SenseElb = 1672;
        // SenseWri = 1504;
        // SenseGri = 1870;
        // BACK
        // SenseBas = 1528;
        // SenseShl = 1796;
        // SenseElb = 2072;
        // SenseWri = 0844;
        // SenseGri = 1870;
        // RELAX
        // SenseBas = 0000;
        // SenseShl = 0000;
        // SenseElb = 0000;
        // SenseWri = 0000;
        // SenseGri = 0000;
     
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
float SenseBasAux = 1500;
float SenseShlAux = 1500;
float SenseElbAux = 1500;
float SenseWriAux = 1500;
float SenseGriAux = 1500;
float SenseBas = 1528;
float SenseShl = 1468;
float SenseElb = 1672;
float SenseWri = 1504;
float SenseGri = 1870;
float teta[4] = {0};
float sense[5] = {0};
 
float L1 = 6.3, L2 = 14.6, L3 = 18.3, L4 = 8.5;
 
float h = 7, t1, t2, t3, t4;
int t = 2000, s = 300;
 
void send_command(void);
void make_and_send_command(void);
void rad2deg(float *ang);
void move(float x, float y, float z, float phi);
void calc_tetas(float x, float y, float z, float phi);

 
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
 
            //system ("clear");
 
            printf("Digite space para sair\n");
 
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
            //printf("#0P%dS%d#1P%dS%d#2P%dS%d#3P%dS%d#4P%dS%dT%d\n", (int)SenseBas, s,  (int)SenseShl, s,  (int)SenseElb, s,  (int)SenseWri, s,  (int)SenseGri, s, t);
            printf("%s\n", last_comando);
            printf("BASE     -> (Q)ESQUERDA ; (A)DIREITA  | LARGURA DO PULSO: %d \t | ANGULO: %d | ANGULOC: %f \n", (int)SenseBas, (int)angSenseBas, teta[0]);
            printf("OMBRO    -> (W)CIMA     ; (S)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %d | ANGULOC: %f \n", (int)SenseShl, (int)angSenseShl, teta[1]);
            printf("COTOVELO -> (E)CIMA     ; (D)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %d | ANGULOC: %f \n", (int)SenseElb, (int)angSenseElb, teta[2]);
            printf("PUNHO    -> (R)CIMA     ; (F)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %d | ANGULOC: %f \n", (int)SenseWri, (int)angSenseWri, teta[3]);
            printf("GARRA    -> (T)FECHAR   ; (G)ABRIR    | LARGURA DO PULSO: %d \t              \n", (int)SenseGri);
            printf("Digite space para sair\n");
            if(system("/bin/stty raw"));
            c = getchar();
            if(system("/bin/stty cooked"));
 
 
            if (c == 'q') {
                SenseBas = SenseBas + 4;
                SenseBasAux = SenseBasAux + 4;
                pos = SenseBas;
                angSenseBas = (636- SenseBasAux) * 0.1041667;
                sprintf(comando, "#%dP%d", BAS_SERVO, trava(BAS_SERVO, pos));
            } else if (c == 'a') {
                SenseBas = SenseBas - 4;
                SenseBasAux = SenseBasAux - 4;
                pos = SenseBas;
                angSenseBas = (636 - SenseBasAux ) * 0.1041667;
                sprintf(comando, "#%dP%d", BAS_SERVO, trava(BAS_SERVO, pos));
 
            } else if (c == 'w') {
                SenseShl = SenseShl + 4;
                SenseShlAux = SenseShlAux + 4;
                pos = SenseShl;
                angSenseShl = (SenseShlAux - 712) * 0.1142132;
                //angSenseShl = (SenseShlAux - 500) * 0.09;
                sprintf(comando, "#%dP%d", SHL_SERVO, pos * (pos < 1850) + 1850 * (pos >= 1850));
            } else if (c == 's') {
                SenseShl = SenseShl - 4;
                SenseShlAux = SenseShlAux - 4;
                pos = SenseShl;
                angSenseShl = (SenseShlAux - 712) * 0.1142132;
                // angSenseShl = (SenseShlAux - 500) * 0.09;
                sprintf(comando, "#%dP%d", SHL_SERVO, pos * (pos < 1850) + 1850 * (pos >= 1850));
            } else if (c == 'e') {
                SenseElb = SenseElb - 4;
                SenseElbAux = SenseElbAux - 4;
                pos = SenseElb;
                angSenseElb = (2256 - SenseElbAux) * 0.1209677 - 180;
                sprintf(comando, "#%dP%d", ELB_SERVO, pos);
            } else if (c == 'd') {
                SenseElb = SenseElb + 4;
                SenseElbAux = SenseElbAux + 4;
                pos = SenseElb;
                angSenseElb = (2256 - SenseElbAux) * 0.1209677 - 180;
                sprintf(comando, "#%dP%d", ELB_SERVO, pos);
            } else if (c == 'r') {
                SenseWri = SenseWri + 4;
                SenseWriAux = SenseWriAux + 4;
                pos = SenseWri;
                angSenseWri = (SenseWriAux - 584) * 0.0982533 - 90;
                // angSenseWri = (SenseWri - 500) * 0.0982533;
                sprintf(comando, "#%dP%d", WRI_SERVO, trava(WRI_SERVO, pos));
            } else if (c == 'f') {
                SenseWri = SenseWri - 4;
                SenseWriAux = SenseWriAux - 4;
                pos = SenseWri;
                angSenseWri = (SenseWriAux - 584) * 0.0982533 - 90;
                // angSenseWri = (SenseWri - 500) * 0.0982533;
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
                fprintf(f2, "#0P%uT1000#1P%uT1000#2P%uT1000#3P%uT1000#4P%uT1000\n", (unsigned int)SenseBas, (unsigned int)SenseShl, (unsigned int)SenseElb, (unsigned int)SenseWri, (unsigned int)SenseGri);
                fprintf(f2, "%u,%u,%u,%u,%u\n", (unsigned int)SenseBasAux, (unsigned int)SenseShlAux, (unsigned int)SenseElbAux, (unsigned int)SenseWriAux, (unsigned int)SenseGriAux);
                 
                fclose(f2);
            } else if (c == '1') {
                sprintf(comando, "%s", POS1);
            } else if (c == '2') {
                sprintf(comando, "%s", POS2);               
            } else if (c == '3') {
                sprintf(comando, "%s", POS3);               
            } else if (c == '4') {
                sprintf(comando, "%s", POS4);               
            } else if (c == '5') {
                sprintf(comando, "%s", POS5);               
            } else if (c == '6') {
                sprintf(comando, "%s", POS6);
            } else if (c == '7') {
                sprintf(comando, "%s", HOME_POS);               
            } else if (c == '0') {
                // SenseBas = 1528;
                // SenseShl = 1796;
                // SenseElb = 2072;
                // SenseWri = 844;
                // SenseGri = 1870;
                // // send_command(1528,1796,2072,844,1870);
                // int delay = 2000;
                // sprintf(comando,"#0P%dT%d#1P%dT%d#2P%dT%d#3P%dT%d#4P%dT%d", SenseBas, s,  SenseShl, s,  SenseElb, s,  SenseWri, s,  SenseGri, s, t);
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
            } else if (c == '8') {
                sprintf(comando, "#4P%d", trava(GRI_SERVO, 1350));
            } else if (c == '9') {
                sprintf(comando, "#4P%d", trava(GRI_SERVO, 1890));
            } else if (c == 'h') {
                printf("Insira s e t: ");
                if(scanf("%d", &s));
                if(scanf("%d", &t));
 
                sprintf(comando, "#0P1528S%d#1P1468S%d#2P1672S%d#3P1504S%d#4P1870S%dT%d", s, s, s, s, s, t);
            } else if (c == 'z') {
 
                sprintf(comando, "#0P1540S300#1P1244S300#2P1716S300#3P1720S300#4P1150S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1540S300#1P1244S300#2P1716S300#3P1720S300#4P1160S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1540S300#1P1148S300#2P1596S300#3P1720S300#4P1160S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1532S300#1P1148S300#2P1596S300#3P1720S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1532S300#1P1232S300#2P1596S300#3P1732S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1448S300#1P1548S300#2P2144S300#3P1844S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1448S300#1P1548S300#2P2176S300#3P1916S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
                sprintf(comando, "#0P1400S300#1P1364S300#2P1996S300#3P1916S300#4P1980S300T4000");
                enviar_comando(comando, serial_fd);
                memset(comando, 0, BUFSIZE);
                printf("Pressione enter para acordar...");
                getchar();
            } else if (c == 'm'){
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
    sprintf(comando, "#0P%uS300#1P%uS300#2P%uS300#3P%uS300\n", (unsigned int)sense[0], (unsigned int)sense[1], (unsigned int)sense[2], (unsigned int)sense[3]);
    send_command();
}
 
void rad2deg(float *ang){
    *ang *= 180/PI;
}
 
void move(float x, float y, float z, float phi){
    calc_tetas(x, y, z, phi); 
    make_and_send_command();
}

void calc_tetas(float x, float y, float z, float phi) {
    int i;
    phi *= PI/180;
    float exy = sqrt(pow(x,2) + pow(y,2));
    teta[0] = atan2(y/exy, x/exy);
    float x14 = exy - L4*cos(phi);
    float z14 = z - L1 - L4*sin(phi);
    float c3 = ((pow(x14, 2) + pow(z14, 2) - pow(L2, 2) - pow(L3, 2))/(2*L2*L3));
    float s3 = sqrt(1-pow(c3,2));
    teta[2] = atan2(s3,c3);
    float exz14 = sqrt(pow(x14,2) + pow(z14,2));
    float alpha = atan2(z14/exz14, x14/exz14);
    float beta = atan2(sin(teta[2])*L3/exz14, (L2 + L3*c3)/exz14);
    teta[1] = alpha - beta;
    teta[3] = phi - teta[1] - teta[2];
    for (i = 0; i < 4; i++){
        rad2deg(&teta[i]);
        sense[i] = 0.09*teta[i] + 500;
    }
}