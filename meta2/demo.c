#define HOME_POS "#0P1500#1P1500#2P1500#3P1500#4P1500"
#include "ufrn_al5d.h"
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char** argv){
  char in, comando;
  comando = (char*) malloc(sizeof(char)*BUFSIZE);

  serial_fd = abrir_porta();

  if(serial_fd == -1) {
    printf("Erro abrindo a porta serial /dev/ttyS0\nAbortando o programa...");
    return -1;
  }

  printf("Porta serial /dev/ttyS0 aberta com sucesso\n");

  if(configurar_porta(serial_fd) == -1) {
      printf("Erro inicializando a porta\n");
      close(serial_fd);
      return -1;
  }

  sprintf(comando,"%s",HOME_POS);

  while(in != 'x'){
    memset(comando, 0, BUFSIZE);
    scanf("%s", &in);
    switch (in){
      case 'a':
        sprintf(comando,"#%dP%dT%d",3,trava(3,2000),2000);
        enviar_comando(comando,serial_fd);
        break;
      case 's':
        sprintf(comando,"#%dP%dT%d",2,trava(2,1900),2000);
        enviar_comando(comando,serial_fd);
        break;
      case 'd':
        sprintf(comando,"#%dP%dT%d",1,trava(1,1000),2000);
        enviar_comando(comando,serial_fd);
        break;
      case 'w':
        sprintf(comando,"#%dP%dT%d",4,trava(4,1700),1000);
        enviar_comando(comando,serial_fd);
        break;
    }
  }
  
  return 0;
}