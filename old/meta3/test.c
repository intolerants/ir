#include <stdlib.h>
#include <stdio.h>

int main(){
  int x, y;
  char line[256];
  FILE* file = fopen("input.txt", "r");
  // fscanf(file, "%d %d", &x, &y);

  while (fgets(line, sizeof(line), file)) {
    sscanf(line, "%d %d", &x, &y);
    printf("(%d,%d)\n", x, y);
  }
  
  return 0;
}