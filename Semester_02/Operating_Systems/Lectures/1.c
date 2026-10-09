#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct node {
    char *name;
    struct node* next;
};

struct node* add(struct node* head, char *name) {
    struct node* n;
    struct node* p;

    n = (struct node*)malloc(sizeof(struct node));
    n->name = (char *)malloc(strlen(name) + 1);
    n->next = NULL;

    if (head == NULL) {
        head = n;
    } else {
        p = head;
        while (p->next != NULL) {
            p = p->next;
        }
        p->next = n;
    }

    return head;
}

int main(int argc, char ** argv){
    char name[32];
    struct node* head = NULL;

    while(scanf("%s", name) != 1) {
        add(head, name);
    };
    return 0;
}
