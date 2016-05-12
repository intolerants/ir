#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>

void init(void);
void display(void);

int main(int argc, char** argv){
  glutInit(&argc, argv);
  glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize (600, 600);
  glutInitWindowPosition (100, 100);
  glutCreateWindow ("Desenho do braço");
  init();
  glutDisplayFunc(display);
  glutMainLoop();
  return 0;
}

void init(void){
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glOrtho (0, 600, 0, 600, -1 ,1);
  glColor3f (1.0, 0.0, 0.0);
}

void display(void){
  int x, y, i = 0;
  char line[256];
  FILE* file = fopen("input.txt", "r");
  glClear(GL_COLOR_BUFFER_BIT);
  glBegin(GL_LINES);
  while (fgets(line, sizeof(line), file)) {
    sscanf(line, "%d %d", &x, &y);
    x = (x+50);
    y = (y+50);
    glVertex2i(x,y);
    printf("(%d,%d)\n",x,y);
    if (i++ != 0) glVertex2i(x,y);
  }
  glEnd();
  glFlush();
}