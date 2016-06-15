/*************************************************************/
/*       UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE         */
/*   DEPARTAMENTO DE ENGENHARIA DE COMPUTAÇÃO E AUTOMAÇÃO    */
/*                               */
/*   DRIVER DO BRAÇO ROBÓTICO LYNX AL5D PARA A PORTA SERIAL  */
/*                               */
/*   DESENVOLVEDORES:                        */
/*           - HANOCH        */
/*           - TAYNARA          */
/*************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <inttypes.h> // erro no malloc

extern "C"
{
#include "al5d.h"
}

//Posicao inicial para todos os servos

/*DONT KNOW*/
#define HOME_POS "#0P1345S500#1P1437S500#2P1613S500#3P1503S500#4P1870S500T2000"
#define STANDBY "#0P1345S500#1P1437S500#2P803S500#3P1503S500#4P1870S500T2000"
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
/* Calibracao por regressao linear */
// sense[1] = 762.684+7.948*teta[1];
// sense[2] = 854.377-9.231*teta[2];
// sense[3] = 1465.639+10.114*teta[3];
#define offsetBas 664
#define offsetShl 680
#define offsetElb 2428
#define offsetWri 588
#define gainBas 0.1041667
#define gainShl 0.1142132
#define gainElb 0.1209677
#define gainWri 0.0982533

/*USED*/
#define STEP 0.5
#define NTETAS 5
#define fmax(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

// HOME_POS
// sense[0] = 1528;
// sense[1] = 1468;
// sense[2] = 1672;
// sense[3] = 1504;
// sense[4] = 1870;


double Senses[6][5] = {{1532, 1088, 1612, 844 , 1990},
    {1288, 1344, 2060, 1008, 1990},
    {1832, 1344, 2060, 1008, 1990},
    {1532, 1088, 1612, 844 , 1990},
    {1640, 1180, 1808, 836 , 1990},
    {1476, 1180, 1808, 836 , 1990}
};


int serial_fd;
char *comando, *last_comando;

int relax_flag = 0;

char c = 'a';

double X, Y, Z;
double x, y, z, phi = 0.0;

double teta[5] = { -90 , 90, -90, 0, 0};
int sense[5] = {1528, 1468, 1672, 1504, 1870};

double L1 = 6.3, L2 = 14.6, L3 = 18.3, L4 = 8.5;

double h = 7, t1, t2, t3, t4;
int t = 2000, s = 300;

double *path;

void send_command(void);
void make_and_send_command(void);
void rad2deg(double *ang);
void calc_direta(void);
void calc_tetas(double x, double y, double z, double phi);
void calc_senses(void);
void move(double x, double y, double z, double phi);
void move_step_by_step(double x, double y, double z, double phi);
void pega(void);
void solta(void);
void repouso(void);
void posaParaFoto(void);
void pega_piloto(void);
void desenha(int *bolas, int n);
void magica(void);
void pega_piloto_custom(double alvo[7][3]);
void desenha_targets(double alvo[][3], int *bolas, int n_bolas);
void read_coords(void);
void read_bolas(int *bolas, int *n_bolas);
double pixelX2cmY(double x);
double pixelY2cmX(double x);
void readPath(void);
void fechaNoPipe(void);
void pegaPipe(double x, double y);
void juma(void);


int main()
{
    //ufrn_header();


    // INICIO DO PROGRAMA DEMO //

    printf("\n\n+++++++PROGRAMA INTOLERANTS INICIADO+++++++\n\n");

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

        sleep(2);
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

            if (system ("clear"));

            calc_direta();

            // Calcula a inversa pelas posicoes retornadas pela direta atraves dos angulos de junta
            // calc_tetas(X, Y, Z, phi);

            printf("A coordenada x do ponto:\t %.2lf \n", x);
            printf("A coordenada y do ponto:\t %.2lf \n", y);
            printf("A coordenada z do ponto:\t %.2lf \n", z);
            printf("O angulo phi da ferramenta:\t %2.f\n", teta[1] + teta[2] + teta[3]);
            //printf("#0P%dS%d#1P%dS%d#2P%dS%d#3P%dS%d#4P%dS%dT%d\n", (int)sense[0], s,  (int)sense[1], s,  (int)sense[2], s,  (int)sense[3], s,  (int)sense[4], s, t);
            printf("%s\n", last_comando);
            printf("BASE     -> (Q)ESQUERDA ; (A)DIREITA  | LARGURA DO PULSO: %d \t | ANGULO: %.2lf \n", sense[0], teta[0]);
            printf("OMBRO    -> (W)CIMA     ; (S)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %.2lf \n", sense[1], teta[1]);
            printf("COTOVELO -> (E)CIMA     ; (D)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %.2lf \n", sense[2], teta[2]);
            printf("PUNHO    -> (R)CIMA     ; (F)BAIXO    | LARGURA DO PULSO: %d \t | ANGULO: %.2lf \n", sense[3], teta[3]);
            printf("GARRA    -> (T)FECHAR   ; (G)ABRIR    | LARGURA DO PULSO: %d \t              \n", sense[4]);
            printf("Digite space para sair\n");
            if (system("/bin/stty raw"));
            c = getchar();
            if (system("/bin/stty cooked"));


            if (c == 'q' || c == 'a') {
                if (c == 'q')
                    teta[0] -= STEP;
                else if (c == 'a')
                    teta[0] += STEP;
                calc_senses();
                // sense[0] = offsetBas - teta[0] / gainBas;
                sprintf(comando, "#%dP%d", BAS_SERVO, sense[0]);
            } else if (c == 'w' || c == 's') {
                if (c == 'w')
                    teta[1] += STEP;
                else if (c == 's')
                    teta[1] -= STEP;
                // sense[1] = teta[1] / gainShl + offsetShl;
                calc_senses();
                sprintf(comando, "#%dP%d", SHL_SERVO, sense[1]);
            } else if (c == 'e' || c == 'd') {
                if (c == 'e')
                    teta[2] += STEP;
                else if (c == 'd')
                    teta[2] -= STEP;
                // sense[2] = -(teta[2] + 180) / gainElb + offsetElb;
                calc_senses();
                sprintf(comando, "#%dP%d", ELB_SERVO, sense[2]);
            } else if (c == 'r' || c == 'f') {
                if (c == 'r')
                    teta[3] += STEP;
                else if (c == 'f')
                    teta[3] -= STEP;
                // sense[3] = (teta[3] + 90) / gainWri + offsetWri;
                calc_senses();
                sprintf(comando, "#%dP%d", WRI_SERVO, sense[3]);
            } else if (c == 't' || c == 'g') {
                if (c == 't')
                    sense[4] += 10;
                else if (c == 'g')
                    sense[4] -= 10;
                sprintf(comando, "#%dP%d", GRI_SERVO, sense[4]);
            } else if (c == 'p') {
                if (system("gnome-terminal -x /home/aluno/intolerants/meta3/meta3"));
            } else if (c == '=') {
                FILE *f = fopen("input.txt", "a");
                if (f == NULL)
                {
                    printf("Error opening file!\n");
                    exit(1);
                }

                /* print some text */
                fprintf(f, "%d %d\n", (int)X + OFFSETX, (int)Y + OFFSETY);
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
                if (scanf("%d", &n));
                bolas = ((int*) malloc(n * sizeof(int)));
                printf("\nDigite a ordem das bolinhas:\n");
                for (i = 0; i < n; i++)
                    if (scanf("%d", &bolas[i]));
                //bolas[i] = i+1;
                desenha(bolas, n);
            } else if (c == 'o') {
                c = '0';
                magica();
            } else if (c == 'j') {
                c = '0';
                juma();
                // } else if (c == '0') {
                // repouso();
            } else if (c == '1') {
                move(2.97, -22.55, 9, -5);
                sleep(3);
                move(6.02, -27.16, 9.63, -10.38);
                sleep(3);
            } else if (c == 'h') {
                printf("Insira s e t: ");
                if (scanf("%d", &s));
                if (scanf("%d", &t));

                sprintf(comando, "#0P1528S%d#1P1468S%d#2P1672S%d#3P1504S%d#4P1870S%dT%d", s, s, s, s, s, t);
            } else if (c == 'z') {
                pega();
                // getchar();
            } else if (c == 'x') {
                solta();
                // printf("solta\n");
                // getchar();
            } else if (c == 'm') {
                read_coords();
                move(x, y, z, phi);
                // printf("%s\n", comando);
            } else if (c == 'n') {
                read_coords();
                move_step_by_step(x, y, z, phi);
            } else if (c == 'l') {
                solta();
                sleep(3);
                posaParaFoto();
            }
            if (enviar_comando(comando, serial_fd) != -1)
            {
                printf("\nEnviando de comando.\n");
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
        sleep(4);
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
    enviar_comando(comando, serial_fd);
    sprintf(last_comando, "%s", comando);
    memset(comando, 0, BUFSIZE);
}

void make_and_send_command(void) {
    sprintf(comando, "#0P%uS300#1P%uS300#2P%uS300#3P%uS300\n", (unsigned int)sense[0], (unsigned int)sense[1], (unsigned int)sense[2], (unsigned int)sense[3]);
    send_command();
}

void rad2deg(double *ang) {
    *ang *= 180 / PI;
}

void calc_direta(void) {
    // Converte angulos de junta para radiano
    t1 = teta[0] * PI / 180;
    t2 = teta[1] * PI / 180;
    t3 = teta[2] * PI / 180;
    t4 = teta[3] * PI / 180;

    // Calcula a direta pelos angulos de junta
    x = cos(t1) * (L3 * cos(t2 + t3) + L2 * cos(t2) + L4 * cos(t2 + t3 + t4));
    y = sin(t1) * (L3 * cos(t2 + t3) + L2 * cos(t2) + L4 * cos(t2 + t3 + t4));
    z = L1 + L3 * sin(t2 + t3) + L2 * sin(t2) + L4 * sin(t2 + t3 + t4);
}

void calc_tetas(double x, double y, double z, double phi) {
    int i;
    printf("\nVou calcular tetas para posicao - x:%.2lf y:%.2lf z:%.2lf phi%.2lf\n", x, y, z, phi);
    phi *= PI / 180;
    double exy = sqrt(pow(x, 2) + pow(y, 2));
    teta[0] = atan2(y / exy, x / exy);
    double x14 = exy - L4 * cos(phi);
    double z14 = z - L1 - L4 * sin(phi);
    double c3 = ((pow(x14, 2) + pow(z14, 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3));
    double s3 = -sqrt(fabs(1 - pow(c3, 2)));
    // double s3 = -sqrt(1-pow(c3,2));
    teta[2] = atan2(s3, c3);
    double exz14 = sqrt(pow(x14, 2) + pow(z14, 2));
    double alpha = atan2(z14 / exz14, x14 / exz14);
    double beta = atan2(sin(teta[2]) * L3 / exz14, (L2 + L3 * c3) / exz14);
    teta[1] = alpha - beta;
    teta[3] = phi - teta[1] - teta[2];
    printf("\nexy:%.2lf x14:%.2lf s3sqrt:%.2lf exz14:%.2lf\n", exy, x14, 1 - pow(c3, 2), exz14);
    for (i = 0; i < 4; i++) {
        rad2deg(&teta[i]);
        // printf("%d) %.2lf\n", i+1, teta[i]);
    }
}

void calc_senses() {

    /* Braco 16.06.15*/
    //  6,34922111715742e-06    -0,118223188700800  57,5242890711109
    // -6,62510896668430e-06   0,128033175274047   -80,3030282309799
    //  1,56089622201365e-05    -0,148822363834961  109,439558839270
    //  2,65804863506011e-06    0,0906516854792717  -142,254039064384

    double coefInv[4][3] = {{0.00623456790123457,   -8.82777777777778,  500.000000000000},
        {0.00444444444444433,   8.35555555555557,   649.000000000000},
        {0.0149794238683127,    -7.65185185185187,  803.000000000000},
        { -0.00277777777777780,  10.1500000000000,   1503}
    };
    int i, j;
    for (i = 0; i < 4; i++) {
        sense[i] = 0;
        for (j = 0; j < 3; j++){
            sense[i] += coefInv[i][j] * pow(teta[i], 2 - j);
        }
    }
    /* Braco Velho*/
    // sense[0] = -(teta[0] / 0.1041667 - 636 - 28);

    // /* Calibracao manual*/
    // // sense[1] = teta[1]/0.1142132 - 32 + 712;
    // // sense[2] = -((teta[2]+180)/0.1209677 - 2256 - 172);
    // // sense[3] = (teta[3]+90)/0.0982533 + 4 + 584;

    // /*Era a boa*/
    // /* Calibracao por regressao linear */
    // sense[1] = 762.684 + 7.948 * teta[1];
    // sense[2] = 854.377 - 9.231 * teta[2];
    // sense[3] = 1465.639 + 10.114 * teta[3];

    // /* Calibracao quadrada*/
    // // sense[1] = 0.0020*pow(teta[1],2) + 7.6037*teta[1] + 775.4128;
    // // sense[2] = -0.0088*pow(teta[2],2) -10.9082*teta[2] + 783.4413;
    // // sense[3] = 10.6*teta[3] + 1461.8;

    // /* Calibracao de quarta ordem */
    // // sense[1] = -9.16283572805788 * pow(10, -06) * pow(teta[1], 4) + 0.00249238051918379 * pow(teta[1], 3) - 0.220292028854092 * pow(teta[1], 2) + 14.8408844789694 * teta[1] + 712.940479709510;
    // // sense[2] = 2.95031078343459 * pow(10, -06) * po0w(teta[2], 4) + 0.00133012241558227 * pow(teta[2], 3) + 0.204563648896187 * pow(teta[2], 2) + 3.35185281794406 * teta[2] + 1113.04750020549;
    // // sense[3] = -0.0485371397804810 * pow(teta[3], 4) + 3.17374947124019 * pow(teta[3], 3) - 74.7891701648607 * pow(teta[3], 2) + 758.694940993243 * teta[3] - 1195.19947031368;

    // /* Calibracao tay */
    // // sense[1] = 680.022+8.755*teta[1];
    // // sense[2] = 939.983-8.267*teta[2];
    // // sense[3] = 1503.969+10.18*teta[3];


    // // double coefInv[3][5];
    // // coefInv[0][0] = -0.0000104634647;
    // // coefInv[0][1] = 0.0041262617990;
    // // coefInv[0][2] = -0.5880722281595;
    // // coefInv[0][3] = 43.7376623927511;
    // // coefInv[0][4] = -100.6774234162458;

    // // coefInv[1][0] = 0.000019878894;
    // // coefInv[1][1] = 0.007765778176;
    // // coefInv[1][2] = 1.101645705756;
    // // coefInv[1][3] = 57.071877411949;
    // // coefInv[1][4] = 2220.584002136401;

    // // coefInv[2][0] = 0.182790109972;
    // // coefInv[2][1] = -6.566632619850;
    // // coefInv[2][2] = 76.275365447122;
    // // coefInv[2][3] = -291.855614634314;
    // // coefInv[2][4] = 1751.070894676433;

    // // int i, j;
    // // for (i = 0; i < 3; i++){
    // //     sense[i+1] = 0;
    // //     for (j = 0; j < 5; j++)
    // //         sense[i+1] += coefInv[i][j]*pow(teta[i+1], 4-j);
    // // }

    // // double coefInv[3][5];
    // // coefInv[0][0] = -0.0000130074643;
    // // coefInv[0][1] = 0.0048936140211;
    // // coefInv[0][2] = -0.6799625220436;
    // // coefInv[0][3] = 49.1398179698763;
    // // coefInv[0][4] = -200.0835952186753;

    // // coefInv[1][0] = -0.000031383909;
    // // coefInv[1][1] = -0.012030352524;
    // // coefInv[1][2] = -1.666970797272;
    // // coefInv[1][3] = -108.311783997613;
    // // coefInv[1][4] = -1337.002954744356;

    // // coefInv[2][0] = 0.295869119321;
    // // coefInv[2][1] = -10.568457591541;
    // // coefInv[2][2] = 120.936917083721;
    // // coefInv[2][3] = -450.788951491756;
    // // coefInv[2][4] = 1847.803237565521;

    // // int i, j;
    // // for (i = 0; i < 3; i++){
    // //     sense[i+1] = 0;
    // //     for (j = 0; j < 5; j++)
    // //         sense[i+1] += coefInv[i][j]*pow(teta[i+1], 4-j);
    // // }
}

void move(double x, double y, double z, double phi) {
    // printf("\nMOVE - x:%.2lf y:%.2lf z:%.2lf phi%.2lf\n", x, y, z, phi);
    X = x;
    Y = y;
    Z = z;
    calc_tetas(x, y, z, phi);
    // int i;
    // for (i = 0; i < 4; i++){
    //     printf("teta[%d]: %.2lf ", i+1, teta[i]);
    // }
    // printf("\n");
    calc_senses();
    make_and_send_command();
    // teta[0] = (offsetBas - sense[0]) * gainBas;
    // teta[1] = (sense[1] - offsetShl) * gainShl;
    // teta[2] = (offsetElb - sense[2]) * gainElb - 180;
    // teta[3] = (sense[3] - offsetWri) * gainWri - 90;

}

void move_step_by_step(double x, double y, double z, double phi) {

    // interpola o movimento
    //z += (0.031400685275072*pow(y,2) + 1.965132928224196*y + 41.678905898998096) - 13;
    // z +=  (0.12608*y+16.03803) - 13;
    // printf("_____________________\nz:%.2lf\n", z);
    // if (y < -24) {
    //     if (y < -28)
    //         z += -2.5;
    //     else if (y < -25)
    //         z += -1.3;
    //     else if (y < -21)
    //         z += -1;
    // }
    printf("\nORIGEN TO-> - X:%.2lf Y:%.2lf Z:%.2lf phi%.2lf\n", X, Y, Z, phi);
    printf("\nMOVE-STEP-BY-STEP TO-> - x:%.2lf y:%.2lf z:%.2lf phi%.2lf\n", x, y, z, phi);
    int i;
    int diffs[3] = {fabs(x - X), fabs(y - Y), fabs(z - Z)};
    int steps = fmax(1, fmax(diffs[0], fmax(diffs[1], diffs[2])));
    steps *= 16;
    double stepSize[3] = {(x - X) / steps, (y - Y) / steps, (z - Z) / steps};
    printf("\nSTEPSIZE(%d)-> - X:%.2lf Y:%.2lf Z:%.2lf\n\n", steps, stepSize[0], stepSize[1], stepSize[2]);
    for (i = 0; i < steps; ++i)
    {
        X += stepSize[0];
        Y += stepSize[1];
        Z += stepSize[2];
        move(X, Y, Z, phi);
        // usleep(10000);
        usleep(30000);
        // usleep(50000);
        // usleep(75000);
        // 7 p r b c g y p
        // 7 r p c b y g r
    }
}
void pega() {
    sprintf(comando, "#4P1950");
    sense[4] = 1950;
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
}

void solta() {
    sprintf(comando, "#4P1320");
    sense[4] = 1320;
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
}

void repouso() {
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

void posaParaFoto() {
    sprintf(comando, "%s", STANDBY);
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
    sleep(3);
}

void pega_piloto() {
    //antes do piloto
    move(0, -32.29, 17.93, 0);
    sleep(3);
    solta();
    sleep(2);
    //pega piloto
    //move(0,-36.94,15.97,-1.09);
    move(0, -37, 15.88, -4.38);

    sleep(3);
    pega();
    sleep(2);
    //acima do piloto
    move(0, -34.72, 24.36, 11.51);
    sleep(3);
    //acima da area de trabalho
    move(0, -26.71, 20.93, 6.5);
    sleep(5);
}

void desenha(int *bolas, int n) {
    int i;
    pega_piloto();
    for (i = 0; i < n; i++) {
        switch (bolas[i]) {
        case 1:
            move(2.97, -22.55, 9.25, -12.96);
            break;
        case 2:
            move(6.02, -27.16, 9.63, -10.38);
            break;
        case 3:
            move(2.94, -31.8, 11.69, -3.07);
            break;
        case 4:
            move(-3.8, -31.6, 11.69, -3.07);
            break;
        case 5:
            move(-6.2, -26.1, 11, -3.07);
            break;
        case 6:
            move(-2.7, -20.3, 9.1, -3.07);
            break;
        }
        sleep(5);
    }
}

void magica() {
    if (system("clear"));

    printf("\nPosando para foto...\n");
    posaParaFoto();

    printf("\nTirando foto e encontrando alvos...\n");
    if (system("./captura/circleFinder"));

    int alvo[7][3];
    double alvoPrecision[7][3];
    int j = 0;
    FILE* file = fopen("pos.txt", "r");
    char line[256];
    while (fgets(line, sizeof(line), file)) {
        // if(sscanf(line, "%d %d %c", &alvo[j][0], &alvo[j][1], &letras[j]));
        if (sscanf(line, "%d %d %c", &alvo[j][0], &alvo[j][1], &alvo[j][2]));
        alvoPrecision[j][0] = -pixelX2cmY(alvo[j][0]);
        alvoPrecision[j][1] = pixelY2cmX(alvo[j][1]);
        alvoPrecision[j][2] = alvo[j][2];
        // move_step_by_step(alvo[j][1], -alvo[j][0], 13, 0);
        // sleep(3);
        j++;
    }
    for (j = 0; j < 7; ++j)
    {
        printf("%d) %lf %lf %c\n", j, alvoPrecision[j][0], alvoPrecision[j][1], (char)alvoPrecision[j][2]);
        // printf("%d) %d %d %c\n", j, alvo[j][0], alvo[j][1], letras[j]);
    }
    int *bolas, n_bolas, i;

    printf("\nDigite quantas bolinhas:\n");
    if (scanf("%d", &n_bolas));
    // n_bolas = 7;
    bolas = ((int*) malloc(n_bolas * sizeof(int)));

    read_bolas(bolas, &n_bolas);

    desenha_targets(alvoPrecision, bolas, n_bolas);
}

void desenha_targets(double alvo[][3], int *bolas, int n_bolas) {
    int i, j;
    pega_piloto_custom(alvo);
    for (i = 0; i < n_bolas; i++) {
        for (j = 0; j < 7; ++j) {
            if (((char)bolas[i]) == ((char)alvo[j][2])) {
                move_step_by_step(alvo[j][1], alvo[j][0], 13, 0);
                sleep(2);
                break;
            }
        }
    }
}
void pega_piloto_custom(double alvo[7][3]) {
    move(0, -26.8, 20.9, 0);

    int i = 0;
    for (i = 0; i < 7; ++i)
    {
        printf("%d) %lf %lf %c\n", i, alvo[i][0], alvo[i][1], (char)alvo[i][2]);
    }
    // p r b y c g
    sleep(3);
    i = 0;
    printf("Procurando piloto\n");
    while (((char)alvo[i][2]) != 't') {
        // printf("alvo[%d][2] = %c\n", i++, (char)alvo[i][2]);
        i++;
        if (i > 6) {
            printf("Piloto nao encontrado\n");
            exit(0);
        }
    }
    printf("Piloto encontrado em %d\n", i);
    printf("Preparando para pegar o piloto...\n");
    move(alvo[i][1], alvo[i][0] + 3, 25, 0);
    printf("Abrindo garra...\n");
    sleep(3);
    solta();
    printf("Descendo...\n");
    sleep(1);
    move_step_by_step(alvo[i][1], alvo[i][0], 14.9, 0);
    printf("Fechando garra...\n");
    sleep(2);
    pega();
    printf("Subindo com o piloto...\n");
    sleep(1);
    move_step_by_step(alvo[i][1], alvo[i][0], 25, 5);
    sleep(2);
    move_step_by_step(0, -26.8, 25, 0);
    sleep(2);

}

void read_coords() {
    X = x;
    Y = y;
    Z = z;
    printf("\n\n");
    printf("x: ");
    if (scanf("%lf", &x));
    printf("y: ");
    if (scanf("%lf", &y));
    printf("z: ");
    if (scanf("%lf", &z));
    printf("phi: ");
    if (scanf("%lf", &phi));
}

void read_bolas(int *bolas, int *n_bolas) {
    int i;
    // printf("\nDigite quantas bolinhas:\n");
    // if (scanf("%d", n_bolas));
    // *n_bolas = 6;
    // bolas = ((int*) malloc(*n_bolas * sizeof(int)));
    printf("\nDigite a ordem das bolinhas:\n");
    printf("\n(p)ink\n(r)ed\n(b)lue\n(c)yan\n(g)reen\n(y)ellow\n");
    // char resposta[7] = {'r', 'c', 'y', 'p', 'g', 'b', 'r'};
    for (i = 0; i < *n_bolas; i++)
        if (scanf(" %c", &bolas[i]));
    // bolas[i] = resposta[i];
}


double pixelX2cmY(double x) {
    return 0.11201 * x + 7.65223;
}

double pixelY2cmX(double x) {
    return -0.12608 * x + 23.18324;
}

void calcPath(void) {
    if (system("clear"));

    printf("\nPosando para foto...\n");
    posaParaFoto();

    printf("\nTirando foto e encontrando obstaculos...\n");
    if (system("./circleFinder"));

}

void readPath(void) {
    FILE* file = fopen("outputResult.txt", "r");
    char line[256];
    double aux;
    int i = 0, size;

    fgets(line, sizeof(line), file);
    if (sscanf(line, "%d", &size));

    path = (double*) malloc(size * 2 * sizeof(double*));

    while (fgets(line, sizeof(line), file)) {
        // if(sscanf(line, "%d %d %c", &alvo[j][0], &alvo[j][1], &letras[j]));
        if (sscanf(line, "%lf %lf", &path[i * 2], &path[i * 2 + 1]));
        aux = -pixelX2cmY(path[i * 2]);
        path[i * 2] = pixelY2cmX(path[i * 2 + 1]);
        path[i * 2 + 1] = aux;
        i++;
    }

    // for (int i = 0; i < size; ++i)
    // {
    //     printf("Pos(%lf,%lf)\n", path[i * 2], path[i * 2 + 1]);
    // }
}

void fechaNoPipe(void) {
    sprintf(comando, "#4P1660");
    sense[4] = 1660;
    enviar_comando(comando, serial_fd);
    memset(comando, 0, BUFSIZE);
}

void pegaPipe(double x, double y) {
    move(0, -26.8, 20.9, 0);
    sleep(2);
    printf("Preparando para pegar o cano...\n");
    move(x, y, 25, -90);
    printf("Abrindo garra...\n");
    sleep(3);
    solta();
    printf("Descendo...\n");
    sleep(1);
    move_step_by_step(x, y, 14.9, -90);
    printf("Fechando garra...\n");
    sleep(1);
    fechaNoPipe();
    printf("Subindo com o cano...\n");
    sleep(1);
    move_step_by_step(x, y, 15.9, -90);
    sleep(1);
}

void juma(void) {
    calcPath();
    readPath();
    pegaPipe(path[0], path[1]);
    // sleep(100);
}