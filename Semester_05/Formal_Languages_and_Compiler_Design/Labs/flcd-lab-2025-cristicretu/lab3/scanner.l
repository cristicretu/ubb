%{
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "symbol_table.h"

typedef struct {
    char *token;
    int st_pos;
} PIFEntry;

typedef struct {
    PIFEntry *entries;
    size_t size;
    size_t capacity;
} PIF;

SymbolTable *identifier_table;
SymbolTable *constant_table;
PIF *pif;
int has_errors = 0;
int line_number = 1;

void pif_init(PIF **p) {
    *p = (PIF *)malloc(sizeof(PIF));
    (*p)->capacity = 100;
    (*p)->size = 0;
    (*p)->entries = (PIFEntry *)malloc((*p)->capacity * sizeof(PIFEntry));
}

void pif_add(PIF *p, const char *token, int st_pos) {
    if (p->size >= p->capacity) {
        p->capacity *= 2;
        p->entries = (PIFEntry *)realloc(p->entries, p->capacity * sizeof(PIFEntry));
    }
    p->entries[p->size].token = strdup(token);
    p->entries[p->size].st_pos = st_pos;
    p->size++;
}

void pif_free(PIF *p) {
    if (p) {
        for (size_t i = 0; i < p->size; i++) {
            free(p->entries[i].token);
        }
        free(p->entries);
        free(p);
    }
}

void add_identifier(const char *id) {
    int pos = symbol_table_get(identifier_table, id);
    if (pos == -1) {
        symbol_table_insert(identifier_table, id);
        pos = symbol_table_get(identifier_table, id);
    }
    pif_add(pif, "IDENTIFIER", pos);
}

void add_constant(const char *constant) {
    int pos = symbol_table_get(constant_table, constant);
    if (pos == -1) {
        symbol_table_insert(constant_table, constant);
        pos = symbol_table_get(constant_table, constant);
    }
    pif_add(pif, "CONSTANT", pos);
}

void add_token(const char *token) {
    pif_add(pif, token, -1);
}

void lexical_error(const char *text) {
    printf("error at line %d: Invalid token '%s'\n", line_number, text);
    has_errors = 1;
}

%}

DIGIT       [0-9]
LETTER      [a-zA-Z]
IDENTIFIER  {LETTER}({LETTER}|{DIGIT}|_)*
NUMBER      {DIGIT}+(\.{DIGIT}+)?
STRING      \"[^\"\n]*\"
COMMENT     #[^\n]*
WHITESPACE  [ \t\r]+
NEWLINE     \n

%%

"program"     { add_token("program"); }
"end"         { add_token("end"); }
"let"         { add_token("let"); }
"if"          { add_token("if"); }
"else"        { add_token("else"); }
"for"         { add_token("for"); }
"fn"          { add_token("fn"); }
"return"      { add_token("return"); }
"read"        { add_token("read"); }
"say"         { add_token("say"); }

{IDENTIFIER}  { add_identifier(yytext); }
{NUMBER}      { add_constant(yytext); }
{STRING}      { add_constant(yytext); }
{COMMENT}     {  }

"<.>"         { add_token("<.>"); }
"//"          { add_token("//"); }
"^^"          { add_token("^^"); }
"%%"          { add_token("%%"); }
"@@"          { add_token("@@"); }
"=="          { add_token("=="); }
"!="          { add_token("!="); }
">="          { add_token(">="); }
"<="          { add_token("<="); }
"|>"          { add_token("|>"); }

"+"           { add_token("+"); }
"-"           { add_token("-"); }
"*"           { add_token("*"); }
"/"           { add_token("/"); }
"%"           { add_token("%"); }
">"           { add_token(">"); }
"<"           { add_token("<"); }
"&"           { add_token("&"); }
"|"           { add_token("|"); }
"^"           { add_token("^"); }
"!"           { add_token("!"); }
"~"           { add_token("~"); }
"="           { add_token("="); }

"("           { add_token("("); }
")"           { add_token(")"); }
"{"           { add_token("{"); }
"}"           { add_token("}"); }
"["           { add_token("["); }
"]"           { add_token("]"); }
";"           { add_token(";"); }
","           { add_token(","); }

{WHITESPACE}  {  }
{NEWLINE}     { line_number++; }

.             { lexical_error(yytext); }

%%

void print_pif() {
    printf("\nProgram Internal Form (PIF)\n");
    printf("%-20s | %s\n", "Token", "ST Position");
    for (size_t i = 0; i < pif->size; i++) {
        if (pif->entries[i].st_pos == -1) {
            printf("%-20s | %s\n", pif->entries[i].token, "-");
        } else {
            printf("%-20s | %d\n", pif->entries[i].token, pif->entries[i].st_pos);
        }
    }
}

void print_symbol_table(const char *name, SymbolTable *st) {
    printf("\nSymbol Table: %s\n", name);
    printf("%-10s | %s\n", "Position", "Value");
    
    size_t size;
    HashEntry **sorted = symbol_table_get_sorted(st, &size);
    
    for (size_t i = 0; i < size; i++) {
        if (sorted[i]->is_occupied) {
            printf("%-10zu | %s\n", sorted[i]->value, sorted[i]->key);
        }
    }
    
    free(sorted);
}

int yywrap() {
    return 1;
}

int main(int argc, char **argv) {
    FILE *input = NULL;
    
    if (argc > 1) {
        input = fopen(argv[1], "r");
        if (!input) {
            fprintf(stderr, "Error: Cannot open file '%s'\n", argv[1]);
            return 1;
        }
        yyin = input;
    }
    
    identifier_table = symbol_table_new(16);
    constant_table = symbol_table_new(16);
    pif_init(&pif);
    
    yylex();
    
    if (has_errors) {
        printf("\n errors!\n");
    } else {
        printf("completed!\n");
    }
    
    print_pif();
    print_symbol_table("Identifiers", identifier_table);
    print_symbol_table("Constants", constant_table);
    
    pif_free(pif);
    symbol_table_free(identifier_table);
    symbol_table_free(constant_table);
    
    if (input) {
        fclose(input);
    }
    
    return has_errors ? 1 : 0;
}

