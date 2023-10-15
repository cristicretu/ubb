"""
Manage a list of complex nr

- display all numers to the console
- add a number from the console
- add a random number
- sort by number modulo
- exit the program

"""
import random

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
    # best code here :)
    return f"{z[0] if z[0] != 0 else ''}{'' if z[0] == 0 else '' if z[1] < 0 else '' if z[1] == 0 else '+'}{z[1] if z[1] != 0 else ''}{'' if z[1] == 0 else 'i'}"

def get_modulo(z: tuple) -> float:
    return (z[0] ** 2 + z[1] ** 2) ** 0.5



# fn that implement program requirements

def print_instructions():
    print("1. Add a complex numeber")
    print("2. Display all complex numbers")
    print("3. Add a random complex number")
    print("4. Sort by modulo")
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

    elif cmd == "2":
        print("----------")

        for z in number_list:
            print(to_str(z), end="; ")

        print("\n----------")


    elif cmd == "3":
        real_part = random.randint(-10000, 10000)
        imaginary_part = random.randint(-10000, 10000)

        z_number = create_z(real_part, imaginary_part)
        number_list.append(z_number)

    elif cmd == "4":
        number_list.sort(key=get_modulo)

    elif cmd == "0":
        break

    else:
        print("Invalid command!")
