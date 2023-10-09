"""
Manage a list of complex nr

- display all numers to the console
- add a number from the console
- add a random number
- sort by number modulo
- exit the program

"""

def create_z(real : int, imaginary: int) -> tuple:
    """
    Creates a complex number in the form (real + imag * i)
    :param real: real part of the complex number
    :param imaginary: imaginary part of the complex number
    :return: a tuple representing the complex number
    """
    return real, imaginary

def to_str(z: tuple) -> str:
    """
    Converts a complex number to a string
    :param z: the complex number
    :return: a string representing the complex number
    """
    return f"{z[0]} + {z[1]}i"

def print_instructions():
    print("1. Add a complex numeber")
    print("2. Display all complex numbers")
    print("3. Add a random complex number")
    print("0. Exit")


number_list = []

while True:
    print_instructions()
    cmd = input("Enter command: ")

    if cmd == "1":
        real = int(input("Enter real part: "))
        imaginary = int(input("Enter imaginary part: "))
        z = create_z(real, imaginary)
        number_list.append(z)
        print(to_str(z))

    elif cmd == "2":
        print("----------")
        for z in number_list:
            print(to_str(z))
        print("----------")

    elif cmd == "3":
        pass

    elif cmd == "0":
        break

    else:
        print("Invalid command!")

# fn that implement program requirements
