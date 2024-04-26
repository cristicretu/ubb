- evaluate an arithmetic expression. assume that the expression s correct

    2 + 3 * 4 => 14

    - transform the expression into postfix notation

    2 + 4 => 2 4 +
    4 * 3 + 6  = 4 3 * 6
    4 * (3 + 6) = 4 3 6 + *
    (5+6)*(4-1) => 5 6 + 4 1 - *

Algorithm
- use an auxiliary stack and queue (the result)
- init an empty stack & q
- go through the expr in the infix notation
    - case 1: an operand => queue.push(op)
    - case 2: an ( => stack.push()
    - case 3: an ) => pop from stack, push to queue
    - case 4: an operator => as long as the top of the stack contains an operator w/ a >= priority => pop from stack and push to queue
- when the expr is over, pop everything from the stack, and push in the queue


2 * (4 + 3) - 4 + 6/2 * (1 + 2 * 7) + 9/(1 + 1 * 4 / 2) + 6 + 2 * 8 - 4 * (8/2 + 6 - 4) + 1) + 5c

2 4 3 + * 4 - 6 2 /  1 2 7 * + * + 9 1 1 4 * 2 /  + / 6 + 2 8 * 4 8 2 / 6 + 4 - * - 1 5 +  

Evaluate the expression
- Input is the queue with the polish notation
- GO through the expr
    - Case 1: operand => push to stack
    - Case 2: operator => pop 2 elems from stack, push back the resul

func transform(expr):
    init(q)
    init(s)
    for e in expr:
        if e is operand then
            push(q, e)
        else if e is (
            push(s,e)
        else if e is )
            while top(s) is not (
                v = pop(s)
                push(q,v)
            pop(s)
        else
            while higherOrEquel(top(s), e)
                v = pop(s)
                push(q, v)
            push (s,e)

    while !isEmpty(s)
        v = pop(s)
        push(q, v)

