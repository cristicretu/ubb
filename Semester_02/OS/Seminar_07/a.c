#include <unistd.h>
#include <stdio.h>

int main() {
    char *s[3] = {"A", "B", "C"};
    for (int i = 0; i < 3; ++i) {
        execl("/bin/echo", "echo", s[i], NULL);
    }
}
