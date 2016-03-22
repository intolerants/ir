/*************************************************************/
/*       UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE         */
/*   DEPARTAMENTO DE ENGENHARIA DE COMPUTAÇÃO E AUTOMAÇÃO    */
/*                   */
/*   DRIVER DO BRAÇO ROBÓTICO LYNX AL5D PARA A PORTA SERIAL  */
/*                   */
/*   DESENVOLVEDORES:              */
/*  - ENG. M.SC. DESNES AUGUSTO NUNES DO ROSÁRIO       */
/*  - ENG. DANILO CHAVES DE SOUSA ICHIHARA         */
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

  // INICIO DO PROGRAMA DEMO //

  printf("PROGRAMA DEMONSTRACAO INICIADO\n\n");
  
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

    //////////////////////
    // PRIMEIRO COMANDO //
    //////////////////////
    printf("\nPRIMEIRO COMANDO - POSICAL INICIAL\n");

    sprintf(comando,"%s",HOME_POS);

    //Escrevendo com teste de escrita
    if(enviar_comando(comando,serial_fd)!=-1)
    {
      printf("Enviando de comando com teste de envio: %s\n",HOME_POS);    
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
    printf("\nSEGUNDO COMANDO - MOVER O PUNHO\n");

    printf("Espere 5 segundos...\n");
    sleep(5);

    // NECESSÁRIO: Limpar a regiao de memória da string //
    //             antes de mandar outro comando        //
    memset(comando, 0, BUFSIZE);

    // TESTE DA FUNCAO TRAVA //
    
    printf("Envio de comando sem teste de envio, utilizando a funcao trava: %s\n",comando);
    
    sprintf(comando,"#%dP%dT%d",3,trava(3,2000),2000);
    enviar_comando(comando,serial_fd);

    printf("Pressione enter para continuar...");
    getchar();

    //////////////////////
    // TERCEIRO COMANDO //
    //////////////////////
    printf("\nSEGUNDO COMANDO - MOVER O COTOVELO\n");

    printf("Espere 5 segundos...\n");
    sleep(5);

    // NECESSÁRIO: Limpar a regiao de memória da string //
    //             antes de mandar outro comando        //
    memset(comando, 0, BUFSIZE);

    // TESTE DA FUNCAO TRAVA //
    
    printf("Envio de comando sem teste de envio, utilizando a funcao trava: %s\n",comando);
    
    sprintf(comando,"#%dP%dT%d",2,trava(2,1900),2000);
    enviar_comando(comando,serial_fd);

    printf("Pressione enter para continuar...");
    getchar();

    ////////////////////
    // QUARTO COMANDO //
    ////////////////////
    printf("\nSEGUNDO COMANDO - MOVER O OMBRO\n");

    printf("Espere 5 segundos...\n");
    sleep(5);

    // NECESSÁRIO: Limpar a regiao de memória da string //
    //             antes de mandar outro comando        //
    memset(comando, 0, BUFSIZE);

    // TESTE DA FUNCAO TRAVA //
    
    printf("Envio de comando sem teste de envio, utilizando a funcao trava: %s\n",comando);
    
    sprintf(comando,"#%dP%dT%d",1,trava(1,1000),2000);
    enviar_comando(comando,serial_fd);

    printf("Pressione enter para continuar...");
    getchar();

    ////////////////////
    // QUINTO COMANDO //
    ////////////////////
    printf("\nSEGUNDO COMANDO - MOVER A GARRA\n");

    printf("Espere 5 segundos...\n");
    sleep(5);

    // NECESSÁRIO: Limpar a regiao de memória da string //
    //             antes de mandar outro comando        //
    memset(comando, 0, BUFSIZE);

    // TESTE DA FUNCAO TRAVA //
    
    printf("Envio de comando sem teste de envio, utilizando a funcao trava: %s\n",comando);
    
    sprintf(comando,"#%dP%dT%d",4,trava(4,1700),1000);
    enviar_comando(comando,serial_fd);

    printf("Pressione enter para continuar...");
    getchar();

    ///////////////////
    // SEXTO COMANDO //
    ///////////////////
    printf("\nTERCEIRO COMANDO - MOVER O COTOVELO\n");

    printf("Espere 5 segundos...\n");
    sleep(5);

    memset(comando, 0, BUFSIZE);
    sprintf(comando,"#%dP%dT%d",2,trava(2,1500),2000);
    enviar_comando(comando,serial_fd);

    printf("%s\n",comando);

    // FIM DO PROGRAMA DEMO //
          fechar_porta(serial_fd);
    printf("\nAcesso a porta serial /dev/ttyS0 finalizado\n");

      }

  printf("\nPROGRAMA DEMONSTRACAO FINALIZADO\n\n");

  return 0;
}