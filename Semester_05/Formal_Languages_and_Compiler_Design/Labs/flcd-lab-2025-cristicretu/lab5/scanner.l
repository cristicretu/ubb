%{
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "symbol_table.h"
#include "fa.h"

typedef struct {
    char *token;
    int st_pos;
    int bucket_id;
    char *symbol_name;
} PIFEntry;

typedef struct {
    PIFEntry *entries;
    size_t size;
    size_t capacity;
} PIF;

typedef struct {
    char *token;
    int line;
} ErrorEntry;

typedef struct {
    ErrorEntry *entries;
    size_t size;
    size_t capacity;
} ErrorList;

SymbolTable *identifier_table;
SymbolTable *constant_table;
PIF *pif;
ErrorList *errors;
int has_errors = 0;
int line_number = 1;

FiniteAutomaton fa_identifier;
FiniteAutomaton fa_number;
FiniteAutomaton fa_string;

void pif_init(PIF **p) {
    *p = (PIF *)malloc(sizeof(PIF));
    (*p)->capacity = 100;
    (*p)->size = 0;
    (*p)->entries = (PIFEntry *)malloc((*p)->capacity * sizeof(PIFEntry));
}

void errors_init(ErrorList **e) {
    *e = (ErrorList *)malloc(sizeof(ErrorList));
    (*e)->capacity = 50;
    (*e)->size = 0;
    (*e)->entries = (ErrorEntry *)malloc((*e)->capacity * sizeof(ErrorEntry));
}

void errors_add(ErrorList *e, const char *token, int line) {
    if (e->size >= e->capacity) {
        e->capacity *= 2;
        e->entries = (ErrorEntry *)realloc(e->entries, e->capacity * sizeof(ErrorEntry));
    }
    e->entries[e->size].token = strdup(token);
    e->entries[e->size].line = line;
    e->size++;
}

void errors_free(ErrorList *e) {
    if (e) {
        for (size_t i = 0; i < e->size; i++) {
            free(e->entries[i].token);
        }
        free(e->entries);
        free(e);
    }
}

void pif_add(PIF *p, const char *token, int st_pos, int bucket_id, const char *symbol_name) {
    if (p->size >= p->capacity) {
        p->capacity *= 2;
        p->entries = (PIFEntry *)realloc(p->entries, p->capacity * sizeof(PIFEntry));
    }
    p->entries[p->size].token = strdup(token);
    p->entries[p->size].st_pos = st_pos;
    p->entries[p->size].bucket_id = bucket_id;
    p->entries[p->size].symbol_name = symbol_name ? strdup(symbol_name) : NULL;
    p->size++;
}

void pif_free(PIF *p) {
    if (p) {
        for (size_t i = 0; i < p->size; i++) {
            free(p->entries[i].token);
            if (p->entries[i].symbol_name) {
                free(p->entries[i].symbol_name);
            }
        }
        free(p->entries);
        free(p);
    }
}

void add_identifier(const char *id) {
    if (!fa_validate_string(&fa_identifier, id)) {
        printf("error at line %d: Invalid identifier '%s' (rejected by FA)\n", line_number, id);
        errors_add(errors, id, line_number);
        has_errors = 1;
        return;
    }
    
    int pos = symbol_table_get(identifier_table, id);
    if (pos == -1) {
        symbol_table_insert(identifier_table, id);
        pos = symbol_table_get(identifier_table, id);
    }
    int bucket = symbol_table_get_bucket(identifier_table, id);
    pif_add(pif, "IDENTIFIER", pos, bucket, id);
}

void add_constant(const char *constant) {
    if (!fa_validate_string(&fa_number, constant)) {
        printf("error at line %d: Invalid number '%s' (rejected by FA)\n", line_number, constant);
        errors_add(errors, constant, line_number);
        has_errors = 1;
        return;
    }
    
    int pos = symbol_table_get(constant_table, constant);
    if (pos == -1) {
        symbol_table_insert(constant_table, constant);
        pos = symbol_table_get(constant_table, constant);
    }
    int bucket = symbol_table_get_bucket(constant_table, constant);
    pif_add(pif, "CONSTANT", pos, bucket, constant);
}

void add_string_constant(const char *str) {
    if (!fa_validate_string(&fa_string, str)) {
        printf("error at line %d: Invalid string '%s' (rejected by FA)\n", line_number, str);
        errors_add(errors, str, line_number);
        has_errors = 1;
        return;
    }
    
    int pos = symbol_table_get(constant_table, str);
    if (pos == -1) {
        symbol_table_insert(constant_table, str);
        pos = symbol_table_get(constant_table, str);
    }
    int bucket = symbol_table_get_bucket(constant_table, str);
    pif_add(pif, "CONSTANT", pos, bucket, str);
}

void add_token(const char *token) {
    pif_add(pif, token, -1, -1, NULL);
}

void lexical_error(const char *text) {
    printf("error at line %d: Invalid token '%s'\n", line_number, text);
    errors_add(errors, text, line_number);
    has_errors = 1;
}

%}

POTENTIAL_ID       [a-zA-Z_][a-zA-Z0-9_]*
POTENTIAL_NUM      [0-9][0-9.]*
POTENTIAL_STR      \"([^\"\n]|\"\")*\"?

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

{POTENTIAL_ID}   { add_identifier(yytext); }
{POTENTIAL_NUM}  { add_constant(yytext); }
{POTENTIAL_STR}  { add_string_constant(yytext); }
{COMMENT}        {  }

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

void print_pif(FILE *out) {
    fprintf(out, "\nProgram Internal Form (PIF)\n");
    fprintf(out, "%-20s | %-10s | %s\n", "Token", "Bucket", "Symbol Name");
    for (size_t i = 0; i < pif->size; i++) {
        if (pif->entries[i].bucket_id == -1) {
            fprintf(out, "%-20s | %-10s | %s\n", pif->entries[i].token, "-", "-");
        } else {
            fprintf(out, "%-20s | %-10d | %s\n", 
                    pif->entries[i].token, 
                    pif->entries[i].bucket_id,
                    pif->entries[i].symbol_name ? pif->entries[i].symbol_name : "-");
        }
    }
}

void print_symbol_table(const char *name, SymbolTable *st, FILE *out) {
    fprintf(out, "\nSymbol Table: %s\n", name);
    fprintf(out, "%-10s | %-20s | %s\n", "Position", "Value", "Bucket");
    
    size_t size;
    HashEntry **sorted = symbol_table_get_sorted(st, &size);
    
    for (size_t i = 0; i < size; i++) {
        if (sorted[i]->is_occupied) {
            int bucket = symbol_table_get_bucket(st, sorted[i]->key);
            fprintf(out, "%-10zu | %-20s | %d\n", sorted[i]->value, sorted[i]->key, bucket);
        }
    }
    
    free(sorted);
}

void print_errors(FILE *out) {
    if (errors->size > 0) {
        fprintf(out, "\nLexical Errors\n");
        fprintf(out, "%-10s | %s\n", "Line", "Token");
        for (size_t i = 0; i < errors->size; i++) {
            fprintf(out, "%-10d | %s\n", errors->entries[i].line, errors->entries[i].token);
        }
    }
}

int yywrap() {
    return 1;
}

int main(int argc, char **argv) {
    FILE *input = NULL;
    FILE *output = NULL;
    char output_filename[256];
    
    // Initialize finite automata
    fa_init(&fa_identifier, "Identifier");
    fa_init(&fa_number, "Number");
    fa_init(&fa_string, "String");
    
    // Load FA definitions from files
    if (!fa_load_from_file(&fa_identifier, "fa_identifier.txt")) {
        fprintf(stderr, "Error: Failed to load identifier FA\n");
        return 1;
    }
    
    if (!fa_load_from_file(&fa_number, "fa_number.txt")) {
        fprintf(stderr, "Error: Failed to load number FA\n");
        return 1;
    }
    
    if (!fa_load_from_file(&fa_string, "fa_string.txt")) {
        fprintf(stderr, "Error: Failed to load string FA\n");
        return 1;
    }
    
    printf("Loaded Finite Automata:\n");
    printf("- Identifier FA: %zu states, %zu transitions\n", 
           fa_identifier.state_count, fa_identifier.transition_count);
    printf("- Number FA: %zu states, %zu transitions\n", 
           fa_number.state_count, fa_number.transition_count);
    printf("- String FA: %zu states, %zu transitions\n", 
           fa_string.state_count, fa_string.transition_count);
    printf("\n");
    
    if (argc > 1) {
        input = fopen(argv[1], "r");
        if (!input) {
            fprintf(stderr, "Error: Cannot open file '%s'\n", argv[1]);
            return 1;
        }
        yyin = input;
        
        strncpy(output_filename, argv[1], sizeof(output_filename) - 5);
        output_filename[sizeof(output_filename) - 5] = '\0';
        char *dot = strrchr(output_filename, '.');
        if (dot) {
            *dot = '\0';
        }
        strcat(output_filename, ".out");
        
        output = fopen(output_filename, "w");
        if (!output) {
            fprintf(stderr, "Error: Cannot create output file '%s'\n", output_filename);
            fclose(input);
            return 1;
        }
    }
    
    identifier_table = symbol_table_new(16);
    constant_table = symbol_table_new(16);
    pif_init(&pif);
    errors_init(&errors);
    
    yylex();
    
    if (has_errors) {
        printf("\nerrors!\n");
    } else {
        printf("completed!\n");
    }
    
    FILE *out = output ? output : stdout;
    
    print_symbol_table("Identifiers", identifier_table, out);
    print_symbol_table("Constants", constant_table, out);
    print_pif(out);
    print_errors(out);
    
    if (output) {
        printf("Output written to %s\n", output_filename);
    }
    
    pif_free(pif);
    errors_free(errors);
    symbol_table_free(identifier_table);
    symbol_table_free(constant_table);
    
    if (input) {
        fclose(input);
    }
    if (output) {
        fclose(output);
    }
    
    return has_errors ? 1 : 0;
}

