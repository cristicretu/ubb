grammar syntax;

program: PROGRAM statementList END EOF;

statementList: statement*;

statement:
    declStmt
    | inputStmt
    | outputStmt
    | ifStmt
    | forStmt
    | funcDefStmt
    | returnStmt
    | exprStmt;

// Declaration: let x = expr;
declStmt: LET IDENTIFIER ASSIGN expression SEMI
        | LET arrayAccess ASSIGN expression SEMI;

// Input: read x;
inputStmt: READ IDENTIFIER SEMI;

// Output: say expr, expr, ...;
outputStmt: SAY expressionList SEMI;

// If statement
ifStmt: IF LPAREN condition RPAREN block elseBlock?;

elseBlock: ELSE block;

// For loop
forStmt: FOR LPAREN IDENTIFIER ASSIGN expression SEMI condition SEMI IDENTIFIER ASSIGN expression RPAREN block;

// Function definition
funcDefStmt: FN IDENTIFIER LPAREN parameterList? RPAREN block;

// Return statement
returnStmt: RETURN expression SEMI;

// Expression statement
exprStmt: expression SEMI;

// Block
block: LBRACE statementList RBRACE;

// Parameters
parameterList: IDENTIFIER (COMMA IDENTIFIER)*;

// Expression list
expressionList: expression (COMMA expression)*;

// Condition
condition: comparison (logicalOp comparison)*;

comparison: expression comparisonOp expression;

// Expression with precedence
expression: term ((PLUS | MINUS) term)*;

term: factor ((MULT | DIV | FLOORDIV | MOD | GCD | LCM | POWER | DOT) factor)*;

factor: (MINUS)? primary postfixOp?;

postfixOp: FACTORIAL | SIGMOID | RELU;

primary:
    IDENTIFIER
    | NUMBER
    | STRING
    | LPAREN expression RPAREN
    | functionCall
    | vector
    | arrayAccess;

functionCall: IDENTIFIER LPAREN expressionList? RPAREN;

vector: LBRACKET expressionList? RBRACKET;

arrayAccess: IDENTIFIER LBRACKET expression RBRACKET;

// Operators
comparisonOp: EQ | NEQ | GT | LT | GTE | LTE;

logicalOp: AND | OR | XOR;


// Keywords
PROGRAM: 'program';
END: 'end';
LET: 'let';
READ: 'read';
SAY: 'say';
IF: 'if';
ELSE: 'else';
FOR: 'for';
FN: 'fn';
RETURN: 'return';

// Multi-char operators
DOT: '<.>';
FLOORDIV: '//';
GCD: '^^';
LCM: '%%';
POWER: '@@';
EQ: '==';
NEQ: '!=';
GTE: '>=';
LTE: '<=';
RELU: '|>';

// Single-char operators
PLUS: '+';
MINUS: '-';
MULT: '*';
DIV: '/';
MOD: '%';
GT: '>';
LT: '<';
AND: '&';
OR: '|';
XOR: '^';
FACTORIAL: '!';
SIGMOID: '~';
ASSIGN: '=';

// Delimiters
LPAREN: '(';
RPAREN: ')';
LBRACE: '{';
RBRACE: '}';
LBRACKET: '[';
RBRACKET: ']';
SEMI: ';';
COMMA: ',';

// Literals
NUMBER: [0-9]+ ('.' [0-9]+)?;
STRING: '"' (~["\\\r\n])* '"';
IDENTIFIER: [a-zA-Z_] [a-zA-Z0-9_]*;

// Skip
COMMENT: '#' ~[\r\n]* -> skip;
WS: [ \t\r\n]+ -> skip;
