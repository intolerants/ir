#include <stdlib.h>
#include <stdio.h>

int main() {
  system("rm input.txt");
  system("cat /dev/null > input.txt");
  FILE *f = fopen("input.txt", "w");
  if (f == NULL)
  {
    printf("Error opening file!\n");
    exit(1);
  }

  /* print some text */
  const char *text = "Write this to the file";
  fprintf(f, "Some text: %s\n", text);

  /* print integers and floats */
  int i = 1;
  float py = 3.1415927;
  fprintf(f, "Integer: %d, float: %f\n", i, py);

  /* printing single chatacters */
  char c = 'A';
  fprintf(f, "A character: %c\n", c);

  fclose(f);
  return 0;
}