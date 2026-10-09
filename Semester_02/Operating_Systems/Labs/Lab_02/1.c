#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
  FILE *file = fopen(argv[1], "r");
  char *s = (char *)malloc(100 * sizeof(char));
  fread(s, 1, 100, file);
  printf("%s\n", s);
  free(s);
  return 0;
}
