from random import randint

"""
Write an application that manages a list of rectangles.
Each rectangle is represented using its two opposite corners (x1, y1) and (x2, y2)
The application will have a menu-driven user interface and will provide the following features:

    1. Add a rectangle
        - adds the given rectangle to the list.
        - error if the given rectangle already exists, x1 <= x2 or y1 <= y2

    2. Delete a rectangle
        - deletes the rectangle with the given corners
        - error if non-existing rectangle is given

    3. Show all rectangles
        - shows all rectangles in descending order of their area

    4. Show rectangles that intersect a given one
        - select a rectangle from the list of existing rectangle
        - print those which intersect it by descending order of area

    5. exit
        - exit the program

    Observations:
        - Add 10 random rectangles at program startup
        - Write specifications for non-UI functions
        - Each function does one thing only, and communicates via parameters and return value
        - The program reports errors to the user. It must also report errors from non-UI functions!
        - Make sure you understand the rectangle's representation
        - Try to reuse functions across functionalities (Less code to write and test)
        - Don't use global variables!
"""

def create_rect(x1: int, y1: int, x2: int, y2: int):
    if x1 > x2 or y1 > y2 or x1 == x2 or y1 == y2:
        raise ValueError("Invalid rectangle coordinates")

    # return {"x1": min(x1,x2), "y1": min(y1, y2), "x2": max(x1, x2), "y2": max(y1,y2)}

    return (min(x1,x2), min(y1, y2), max(x1, x2), max(y1,y2))


def generate_rectangles(n: int) -> list:
    rectangles = []
    for _ in range(n):
        x1 = randint(0, 100)
        y1 = randint(0, 100)
        x2 = 1 + randint(x1, 150)
        y2 = 1 + randint(y1, 150)
        rectangles.append(create_rect(x1, y1, x2, y2))

    return rectangles



def to_str(rect) -> str:
    return "({x1}, {y1}), ({x2}, {y2})".format(x1=rect[0], y1=rect[1], x2=rect[2], y2=rect[3])


if __name__ == "__main__":
    rectangles = generate_rectangles(10)

    print(rectangles[0])

    # for rect in rectangles:
    #     print(to_str(rect))

