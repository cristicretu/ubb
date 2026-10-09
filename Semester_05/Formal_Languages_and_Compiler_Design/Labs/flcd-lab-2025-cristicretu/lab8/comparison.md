# LL(1) Parser vs ANTLR Parser Comparison
## Test Program (factorial)
```
program
    read n;
    let fact = 1;
    for (i = 1; i <= n; i = i + 1) {
        let fact = fact * i;
    }
    say fact;
end
```
## TL;DR
| Aspect | My LL(1) Parser | ANTLR |
|--------|-----------------|-------|
| **Algorithm** | LL(1) predictive | ANTLR's ALL(*) |
| **Grammar** | hand-written BNF (81 productions) | .g4 file (auto-generates code) |
| **Parse Tree** | father-sibling table (170 nodes) | S-expression string |
| **Output** | explicit productions + table | nested parentheses |
| **Code** | ~1500 lines C | ~15 lines C++ driver |
## What's the same?
Both parsers accept the same language. Same tokens in, same accept/reject decision out.
## What's different?
### 1. Grammar Format
**LL(1):** Explicit left-factored grammar with epsilon productions
```
stmtList -> stmt stmtList | epsilon
expr -> term exprTail
exprTail -> addOp term exprTail | epsilon
```
**ANTLR:** EBNF shortcuts - cleaner but hides left-recursion handling
```
statementList: statement*;
expression: term ((PLUS | MINUS) term)*;
```
### 2. Parse Tree Representation
**LL(1) (father-sibling table):**
```
Index  | Symbol     | Parent | RightSibling
1      | program    | 0      | 0
2      | PROGRAM    | 1      | 3
3      | stmtList   | 1      | 4
...
```
Space-efficient. Tree traversal = array index lookups.
**ANTLR (S-expression):**
```
(program program (statementList (statement (inputStmt read n ;)) ...))
```
Human-readable inline but annoying to traverse programmatically.
### 3. Derivation Tracking
**LL(1):** Shows every production applied (96 productions for this program)
```
1: program -> PROGRAM stmtList END
2: stmtList -> stmt stmtList
3: stmt -> inputStmt
...
```
You can literally replay the derivation.
**ANTLR:** Just shows what rules matched, no explicit production sequence
```
 -> statementList
 -> statement
 -> inputStmt
```
### 4. Effort
**LL(1):** 
- Hand-coded grammar transformations (left-factoring, epsilon handling)
- Manual FIRST/FOLLOW computation
- Build parse table yourself
- Implement tree construction
**ANTLR:**
- Write grammar, run `antlr4 syntax.g4`
- 4 generated .cpp files appear
- Write a 15-line driver
- Done
## Output Files
- `ll1_output.txt` - My parser's output (272 lines)
- `antlr_output.txt` - ANTLR's output (57 lines)
- `test_program.txt` - The factorial program we parsed
## Bottom Line
ANTLR: ship fast, grammar is spec  
LL(1): understand everything, own the code
