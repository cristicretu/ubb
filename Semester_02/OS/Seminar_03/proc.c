#include <unistd.h>
#include <stdio.h>

int main(int argc, char *argv[]) {
    
     
    fork();
    if (-1 == execl("/bin/ls", "/bin/ls", "-l", NULL)) {
        perror("execl");
        return -1;
    }

    

    if (-1 == execlp("ls", "ls", "-l", NULL)) {
        perror("execlp");
        return -1;
    }
    /* execv("/bin/ls", argv); */
    /* execvp("ls", argv); */

    return 0;
}
