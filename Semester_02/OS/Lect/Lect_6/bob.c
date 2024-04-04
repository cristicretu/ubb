#include <stdio.h>

int main(int argc, char *argv[]) {
  int i;

  FILE *f;

  f = popen("less", "w");

  for (i= 99; i > 0; i--) {
    /* printf("%d bottles of beer on the wall, %d bottles of beer.\n", i, i); */
    /* printf("Take one down and pass it around, %d bottles of beer on the wall.\n\n", i-1); */
    fprintf(f, "%d bottles of beer on the wall, %d bottles of beer.\n", i, i);
    fprintf(f, "Take one down and pass it around, %d bottles of beer on the wall.\n\n", i-1);
  }

  pclose(f);
  return 0;
}
