%{                     
#include <stdio.h>
%}

/*
BNF Grammar:

<program>         ::= "program" <statement_list> "end"
<statement_list>  ::= <statement> <statement_list> | epsilon
<statement>       ::= <decl_stmt> | <input_stmt> | <output_stmt> | <if_stmt> | <for_stmt> | <func_def_stmt> | <return_stmt> | <expr_stmt> | <block_stmt> | <comment>
<comment>         ::= "#" <char_seq>
<block_stmt>      ::= "{" <statement_list> "}"
<decl_stmt>       ::= "let" <identifier> "=" <expression> ";"
<input_stmt>      ::= "read" <identifier> ";"
<output_stmt>     ::= "say" <expression_list> ";"
<if_stmt>         ::= "if" "(" <condition> ")" <block_stmt> <else_part>
<else_part>       ::= "else" <block_stmt> | epsilon
<for_stmt>        ::= "for" "(" <identifier> "=" <expression> ";" <condition> ";" <identifier> "=" <expression> ")" <block_stmt>
<func_def_stmt>   ::= "fn" <identifier> "(" <parameters_opt> ")" <block_stmt>
<return_stmt>     ::= "return" <expression> ";"
<expr_stmt>       ::= <expression> ";"
<parameters_opt>  ::= <parameters> | epsilon
<parameters>      ::= <identifier> <parameters_tail>
<parameters_tail> ::= "," <identifier> <parameters_tail> | epsilon
<expression_list> ::= <expression> <expression_list_tail>
<expression_list_tail> ::= "," <expression> <expression_list_tail> | epsilon
<condition>       ::= <comparison> <condition_rest>
<condition_rest>  ::= <logical_op> <comparison> <condition_rest> | epsilon
<comparison>      ::= <expression> <comparison_op> <expression>
<expression>      ::= <term> <expression_rest>
<expression_rest> ::= <additive_op> <term> <expression_rest> | epsilon
<term>            ::= <factor> <term_rest>
<term_rest>       ::= <multiplicative_op> <factor> <term_rest> | epsilon
<factor>          ::= <primary> "!" | <primary> "~" | <primary> "|>" | <primary> | "-" <primary>
<primary>         ::= <identifier> | <number> | "(" <expression> ")" | <function_call> | <string> | <vector> | <array_access>
<vector>          ::= "[" <vector_elements> "]"
<vector_elements> ::= <expression_list> | epsilon
<array_access>    ::= <identifier> "[" <expression> "]"
<string>          ::= '"' <char_seq> '"'
<char_seq>        ::= <char> <char_seq> | epsilon
<char>            ::= any printable character except '"' or '\n'
<function_call>   ::= <identifier> "(" <arguments_opt> ")"
<arguments_opt>   ::= <expression_list> | epsilon
<additive_op>     ::= "+" | "-"
<multiplicative_op> ::= "*" | "/" | "//" | "%" | "^^" | "%%" | "@@" | "<.>"
<comparison_op>   ::= "==" | "!=" | ">" | "<" | ">=" | "<="
<logical_op>      ::= "&" | "|" | "^"
<identifier>      ::= <letter> <identifier_tail>
<identifier_tail> ::= <letter> <identifier_tail> | <digit> <identifier_tail> | "_" <identifier_tail> | epsilon
<letter>          ::= "a" | "b" | ... | "z" | "A" | "B" | ... | "Z"
<digit>           ::= "0" | "1" | ... | "9"  
<number>          ::= <digit_seq> <decimal_part>
<decimal_part>    ::= "." <digit_seq> | epsilon
<digit_seq>       ::= <digit> <digit_seq_tail>
<digit_seq_tail>  ::= <digit> <digit_seq_tail> | epsilon
*/

DIGIT       [0-9]
LETTER      [a-zA-Z]
IDENTIFIER  {LETTER}({LETTER}|{DIGIT}|_)*
NUMBER      {DIGIT}+(\.{DIGIT}+)?
STRING      \"[^\"\n]*\"
COMMENT     #.*
WHITESPACE  [ \t\n\r]+


%%

"program"     { printf("KEYWORD: %s\n", yytext); }
"end"         { printf("KEYWORD: %s\n", yytext); }
"let"         { printf("KEYWORD: %s\n", yytext); }
"if"          { printf("KEYWORD: %s\n", yytext); }
"else"        { printf("KEYWORD: %s\n", yytext); }
"for"         { printf("KEYWORD: %s\n", yytext); }
"fn"          { printf("KEYWORD: %s\n", yytext); }
"return"      { printf("KEYWORD: %s\n", yytext); }
"read"        { printf("KEYWORD: %s\n", yytext); }
"say"         { printf("KEYWORD: %s\n", yytext); }

{IDENTIFIER}  { printf("IDENTIFIER: %s\n", yytext); }
{NUMBER}      { printf("NUMBER: %s\n", yytext); }
{STRING}      { printf("STRING: %s\n", yytext); }
{COMMENT}     { printf("COMMENT: %s\n", yytext); }

"=="          { printf("COMPARISON_OP: %s\n", yytext); }
"!="          { printf("COMPARISON_OP: %s\n", yytext); }
">="          { printf("COMPARISON_OP: %s\n", yytext); }
"<="          { printf("COMPARISON_OP: %s\n", yytext); }
">"           { printf("COMPARISON_OP: %s\n", yytext); }
"<"           { printf("COMPARISON_OP: %s\n", yytext); }

"<.>"         { printf("MULTIPLICATIVE_OP (dot product): %s\n", yytext); }
"//"          { printf("MULTIPLICATIVE_OP (integer division): %s\n", yytext); }
"^^"          { printf("MULTIPLICATIVE_OP (power): %s\n", yytext); }
"%%"          { printf("MULTIPLICATIVE_OP (modulo): %s\n", yytext); }
"@@"          { printf("MULTIPLICATIVE_OP (concatenation): %s\n", yytext); }
"*"           { printf("MULTIPLICATIVE_OP: (multiplication): %s\n", yytext); }
"/"           { printf("MULTIPLICATIVE_OP (division): %s\n", yytext); }
"%"           { printf("MULTIPLICATIVE_OP (modulo): %s\n", yytext); }

"+"           { printf("ADDITIVE_OP (addition): %s\n", yytext); }
"-"           { printf("ADDITIVE_OP (subtraction): %s\n", yytext); }

"&"           { printf("LOGICAL_OP (bitwise AND): %s\n", yytext); }
"|"           { printf("LOGICAL_OP (bitwise OR): %s\n", yytext); }
"^"           { printf("LOGICAL_OP (bitwise XOR): %s\n", yytext); }

"|>"          { printf("POSTFIX_OP: %s\n", yytext); }
"!"           { printf("POSTFIX_OP: %s\n", yytext); }
"~"           { printf("POSTFIX_OP: %s\n", yytext); }

"="           { printf("ASSIGN_OP: %s\n", yytext); }

"("           { printf("DELIMITER: %s\n", yytext); }
")"           { printf("DELIMITER: %s\n", yytext); }
"{"           { printf("DELIMITER: %s\n", yytext); }
"}"           { printf("DELIMITER: %s\n", yytext); }
"["           { printf("DELIMITER: %s\n", yytext); }
"]"           { printf("DELIMITER: %s\n", yytext); }
";"           { printf("DELIMITER: %s\n", yytext); }
","           { printf("DELIMITER: %s\n", yytext); }

{WHITESPACE}  {  }

.             { printf("UNKNOWN: %s\n", yytext); }

%%

int main(void) {
    yylex();               
    return 0;
}
