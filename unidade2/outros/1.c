

#include <stdio.h>
#include <math.h>
#include <stdlib.h>


#include "ufrn_al5d.h"

#define L1 7.30000000
#define L2 14.6000000
#define L3 19.0000000
#define L4 7.20000000

// Este programa faz o calculo dos angulos atraves de x,y,z e leva o robô para a posição x,y,z.

double o1, o2, o3, o4;

//Posicao inicial para todos os servos
#define HOME_POS "#0P1500#1P1500#2P1500#3P1500#4P1500T10000"

// 02 + 03 + 04 = 360 + fi;
// fi é sempre um valor negativo.


void inversa (double x, double y, double z)
{
    double a,c,d;

    o1= atan2(y,x);
    o3= acos((pow(x,2) + pow(y,2) +pow((z - L1 - L4),2) -pow(L3,2) -pow(L2,2))/(2*L3*L2));

    c= ((z - L1 - L4)*(L3 + L2*cos(o3)) + sin(o3)*L2*sqrt(pow(x,2) + pow(y,2)))/(pow(L3,2) + 2*L3*L2*cos(o3) + pow(L2,2));
    d=(-(z - L1 - L4)*L2*sin(o3) + (L3 + L2*cos(o3))*sqrt(pow(x,2) + pow(y,2)))/(pow(L3,2) + 2*L3*L2*cos(o3) + pow(L2,2));
    a = atan2(c,d);
    o2= a - o3;
    o4= (M_PI_2) -o2 -o3;
}

int main()
{
    ufrn_header();
    int serial_fd;
    char *comando;

    double x,y,z;
    int pbase, pombro, pcotovelo, ppunho, pgarra;

    //Posição inicial.
    //sprintf(comando,"#%dP%d",BAS_SERVO,trava(BAS_SERVO,));
    int i=1;

    while(i!=0)
    {
        //Captura dos valores:
        printf("Digite x: ");
        scanf("%lf",&x);

        printf("Digite y: ");
        scanf("%lf",&y);

        printf("Digite z: ");
        scanf("%lf",&z);

        inversa(x, y, z);

        //Transformação de rad para grau:
        o1= o1*180/(M_PI);
        o2= o2*180/(M_PI);
        o3= o3*180/(M_PI);
        o4= o4*180/(M_PI);

        //Inicializando o Programa:
        serial_fd = abrir_porta();

    if(serial_fd == -1)
    {
        printf("Erro abrindo a porta serial /dev/ttyS0\nAbortando o programa...");
        return -1;
    }
    else
    {
        printf("Porta serial /dev/ttyS0 aberta com sucesso\n");

        if(configurar_porta(serial_fd) == -1)
        {
            printf("Erro inicializando a porta\n");
            close(serial_fd);
            return -1;
        }

        comando = (char*) malloc(sizeof(char)*BUFSIZE);

        //Execução do comando para o braço robótico:
        memset(comando, 0, BUFSIZE);
        //pbase=(-800*o1/90)+ 620;
 	pbase = (o1/0.1) + 1500;
        sprintf(comando,"#%dP%dT3000",BAS_SERVO,trava(BAS_SERVO,pbase));
        enviar_comando(comando,serial_fd);
                         
        memset(comando, 0, BUFSIZE);
       // pombro=(9.01*o2/90)+ 635.3;
	pombro = (o2/(-0.1)) + 530;
        sprintf(comando,"#%dP%dT3000",SHL_SERVO,pombro);
        enviar_comando(comando,serial_fd);

        memset(comando, 0, BUFSIZE);
       // pcotovelo=(-8.72*o3)+893.3;
	pcotovelo = (o3/0.1) + 770;
        sprintf(comando,"#%dP%dT3000",ELB_SERVO,trava(ELB_SERVO,pcotovelo));
        enviar_comando(comando,serial_fd);

        memset(comando, 0, BUFSIZE);
       // ppunho=(9.9479*o4+1471.1);
	ppunho = (o4/0.1) + 540;
        sprintf(comando,"#%dP%dT3000",WRI_SERVO,trava(WRI_SERVO,ppunho));
        enviar_comando(comando,serial_fd);

	printf("\np1: %d", pbase);
	printf("\np2: %d", pombro);
	printf("\np3: %d", pcotovelo);
	printf("\np4: %d", ppunho);



        }
      printf("\n\nPara continuar digite 1, para sair digite 0: ");
      scanf("%d",&i);

    }
	fechar_porta(serial_fd);
    return 0;
}
