%{
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fa.h"

#define HASH_TABLE_SIZE 1024

typedef struct HashNode {
    char token[257];
    int index;
    struct HashNode* next;
} HashNode;

typedef struct {
    HashNode* buckets[HASH_TABLE_SIZE];
    int size;
} HashTable;

HashTable IdentifierTable;
HashTable ConstantTable;

unsigned int hash(const char* str) {
    unsigned long hash = 5381;
    int c;
    while ((c = *str++)) {
        hash = ((hash << 5) + hash) + c;
    }
    return hash % HASH_TABLE_SIZE;
}

void initHashTable(HashTable* ht) {
    for (int i = 0; i < HASH_TABLE_SIZE; i++) {
        ht->buckets[i] = NULL;
    }
    ht->size = 0;
}

int searchHashTable(HashTable* ht, const char* token) {
    unsigned int idx = hash(token);
    HashNode* current = ht->buckets[idx];
    while (current != NULL) {
        if (strcmp(current->token, token) == 0) {
            return current->index;
        }
        current = current->next;
    }
    return -1;
}

int insertHashTable(HashTable* ht, const char* token) {
    int existingIndex = searchHashTable(ht, token);
    if (existingIndex != -1) {
        return existingIndex;
    }
    
    HashNode* newNode = (HashNode*)malloc(sizeof(HashNode));
    strcpy(newNode->token, token);
    newNode->index = ht->size;
    
    unsigned int idx = hash(token);
    newNode->next = ht->buckets[idx];
    ht->buckets[idx] = newNode;
    
    ht->size++;
    return newNode->index;
}

typedef struct {
    char token[257];
    int bucketId;
    char symbolName[257];
} PIF;

PIF ProgramInternalForm[4096];
int pifLength = 0;

int errorFound = 0;
int lineNumber = 1;

typedef struct {
    char token[257];
    int line;
} LexError;

LexError errors[1024];
int errorCount = 0;

/* Finite Automata for validation */
FiniteAutomaton fa_identifier;
FiniteAutomaton fa_number;
FiniteAutomaton fa_string;

void addToPIF(const char* token, int bucketId, const char* symbolName) {
    strcpy(ProgramInternalForm[pifLength].token, token);
    ProgramInternalForm[pifLength].bucketId = bucketId;
    if (symbolName) {
        strcpy(ProgramInternalForm[pifLength].symbolName, symbolName);
    } else {
        ProgramInternalForm[pifLength].symbolName[0] = '\0';
    }
    pifLength++;
}

void addError(const char* token, int line) {
    strcpy(errors[errorCount].token, token);
    errors[errorCount].line = line;
    errorCount++;
}

void addIdentifier(const char* id) {
    if (!fa_validate_string(&fa_identifier, id)) {
        fprintf(stderr, "Lexical error at line %d: Invalid identifier '%s' (rejected by FA)\n", lineNumber, id);
        errorFound = 1;
        addError(id, lineNumber);
        return;
    }
    insertHashTable(&IdentifierTable, id);
    int bucket = hash(id);
    addToPIF("IDENTIFIER", bucket, id);
}

void addNumber(const char* num) {
    if (!fa_validate_string(&fa_number, num)) {
        fprintf(stderr, "Lexical error at line %d: Invalid number '%s' (rejected by FA)\n", lineNumber, num);
        errorFound = 1;
        addError(num, lineNumber);
        return;
    }
    insertHashTable(&ConstantTable, num);
    int bucket = hash(num);
    addToPIF("NUMBER", bucket, num);
}

void addString(const char* str) {
    if (!fa_validate_string(&fa_string, str)) {
        fprintf(stderr, "Lexical error at line %d: Invalid string '%s' (rejected by FA)\n", lineNumber, str);
        errorFound = 1;
        addError(str, lineNumber);
        return;
    }
    insertHashTable(&ConstantTable, str);
    int bucket = hash(str);
    addToPIF("STRING", bucket, str);
}

FILE* outfp = NULL;

void showSymbolTable(const char* name, HashTable* ht) {
    fprintf(outfp, "ST %s (%d entries)\n", name, ht->size);
    
    typedef struct { char token[257]; int index; int bucket; } Entry;
    Entry entries[4096];
    int entryCount = 0;
    
    for (int i = 0; i < HASH_TABLE_SIZE; i++) {
        HashNode* current = ht->buckets[i];
        while (current != NULL) {
            strcpy(entries[entryCount].token, current->token);
            entries[entryCount].index = current->index;
            entries[entryCount].bucket = i;
            entryCount++;
            current = current->next;
        }
    }
    
    for (int i = 0; i < entryCount - 1; i++) {
        for (int j = i + 1; j < entryCount; j++) {
            if (entries[j].index < entries[i].index) {
                Entry temp = entries[i];
                entries[i] = entries[j];
                entries[j] = temp;
            }
        }
    }
    
    for (int i = 0; i < entryCount; i++) {
        fprintf(outfp, "  %d: %s\n", entries[i].index, entries[i].token);
    }
    fprintf(outfp, "\n");
}

void showProgramInternalForm() {
    fprintf(outfp, "PIF (%d tokens)\n", pifLength);
    for (int i = 0; i < pifLength; i++) {
        if (strlen(ProgramInternalForm[i].symbolName) > 0) {
            fprintf(outfp, "  %d | %-12s | %d | %s\n", 
                    i, ProgramInternalForm[i].token, 
                    ProgramInternalForm[i].bucketId,
                    ProgramInternalForm[i].symbolName);
        } else {
            fprintf(outfp, "  %d | %-12s | %d | -\n", 
                    i, ProgramInternalForm[i].token, 
                    ProgramInternalForm[i].bucketId);
        }
    }
    fprintf(outfp, "End PIF\n\n");
}

void showErrors() {
    if (errorCount > 0) {
        fprintf(outfp, "Errors:\n");
        for (int i = 0; i < errorCount; i++) {
            fprintf(outfp, "  line %d: %s\n", errors[i].line, errors[i].token);
        }
        fprintf(outfp, "\n");
    }
}

void freeHashTable(HashTable* ht) {
    for (int i = 0; i < HASH_TABLE_SIZE; i++) {
        HashNode* current = ht->buckets[i];
        while (current != NULL) {
            HashNode* temp = current;
            current = current->next;
            free(temp);
        }
    }
}
%}

POTENTIAL_ID       [a-zA-Z_][a-zA-Z0-9_]*

POTENTIAL_NUM      [0-9][0-9.]*

POTENTIAL_STR      \"([^\"\n]|\"\")*\"?

COMMENT     #[^\n]*
WHITESPACE  [ \t\r]+
NEWLINE     \n

%%

"program"     { addToPIF("PROGRAM", -1, NULL); }
"end"         { addToPIF("END", -1, NULL); }
"let"         { addToPIF("LET", -1, NULL); }
"if"          { addToPIF("IF", -1, NULL); }
"else"        { addToPIF("ELSE", -1, NULL); }
"for"         { addToPIF("FOR", -1, NULL); }
"fn"          { addToPIF("FN", -1, NULL); }
"return"      { addToPIF("RETURN", -1, NULL); }
"read"        { addToPIF("READ", -1, NULL); }
"say"         { addToPIF("SAY", -1, NULL); }

{POTENTIAL_ID}   { addIdentifier(yytext); }
{POTENTIAL_NUM}  { addNumber(yytext); }
{POTENTIAL_STR}  { addString(yytext); }

{COMMENT}        { /* ignore comments */ }

"<.>"         { addToPIF("DOT", -1, NULL); }
"//"          { addToPIF("FLOORDIV", -1, NULL); }
"^^"          { addToPIF("GCD", -1, NULL); }
"%%"          { addToPIF("LCM", -1, NULL); }
"@@"          { addToPIF("POWER", -1, NULL); }
"=="          { addToPIF("EQ", -1, NULL); }
"!="          { addToPIF("NEQ", -1, NULL); }
">="          { addToPIF("GTE", -1, NULL); }
"<="          { addToPIF("LTE", -1, NULL); }
"|>"          { addToPIF("RELU", -1, NULL); }

"+"           { addToPIF("PLUS", -1, NULL); }
"-"           { addToPIF("MINUS", -1, NULL); }
"*"           { addToPIF("MULT", -1, NULL); }
"/"           { addToPIF("DIV", -1, NULL); }
"%"           { addToPIF("MOD", -1, NULL); }
">"           { addToPIF("GT", -1, NULL); }
"<"           { addToPIF("LT", -1, NULL); }
"&"           { addToPIF("AND", -1, NULL); }
"|"           { addToPIF("OR", -1, NULL); }
"^"           { addToPIF("XOR", -1, NULL); }
"!"           { addToPIF("FACTORIAL", -1, NULL); }
"~"           { addToPIF("SIGMOID", -1, NULL); }
"="           { addToPIF("ASSIGN", -1, NULL); }

"("           { addToPIF("LPAREN", -1, NULL); }
")"           { addToPIF("RPAREN", -1, NULL); }
"{"           { addToPIF("LBRACE", -1, NULL); }
"}"           { addToPIF("RBRACE", -1, NULL); }
"["           { addToPIF("LBRACKET", -1, NULL); }
"]"           { addToPIF("RBRACKET", -1, NULL); }
";"           { addToPIF("SEMI", -1, NULL); }
","           { addToPIF("COMMA", -1, NULL); }

{WHITESPACE}  { /* ignore */ }
{NEWLINE}     { lineNumber++; }

.             { errorFound = 1; addError(yytext, lineNumber); fprintf(stderr, "Lexical error at line %d: '%s'\n", lineNumber, yytext); }

%%

int yywrap() { return 1; }

int main(int argc, char** argv) {
    initHashTable(&IdentifierTable);
    initHashTable(&ConstantTable);
    
    fa_init(&fa_identifier, "Identifier");
    fa_init(&fa_number, "Number");
    fa_init(&fa_string, "String");
    
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
    
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <input-file>\n", argv[0]);
        return 1;
    }
    
    FILE *fp = fopen(argv[1], "r");
    if (!fp) {
        perror("Cannot open input file");
        return 1;
    }
    yyin = fp;
    yylex();
    fclose(fp);
    
    system("mkdir -p output");
    
    outfp = fopen("output/pif.txt", "w");
    if (!outfp) {
        perror("Cannot create output/pif.txt");
        return 1;
    }
    showSymbolTable("Identifiers", &IdentifierTable);
    showSymbolTable("Constants", &ConstantTable);
    showProgramInternalForm();
    showErrors();
    fclose(outfp);
    
    if (errorFound) {
        printf("Lexical analysis completed with errors.\n");
    } else {
        printf("Lexical analysis completed successfully.\n");
    }
    printf("Output written to output/pif.txt\n");
    
    freeHashTable(&IdentifierTable);
    freeHashTable(&ConstantTable);
    
    return errorFound ? 1 : 0;
}
