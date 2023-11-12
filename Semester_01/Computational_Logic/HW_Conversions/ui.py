from lib import read_integer_from_keyboard


def print_conversion():
    print("1. Conversion using substitution method; b < h")
    print("2. Conversion using successive divisions method; b > h")
    print("3. Conversion using 10 as an intermediary base; b != 10, h != 10")


def print_menu():
    print("1. Conversion from base b to base h")
    print("2. Operation with two numbers")
    print("0. Exit")


def main():
    print("Author: Cretu Cristian, 913")

    while True:
        print_menu()

        option = read_integer_from_keyboard(">Option: ")

        if option == 1:
            print_conversion()

            number = read_integer_from_keyboard(">Number: ")
            base = read_integer_from_keyboard(">Source Base: ")
            target_base = read_integer_from_keyboard(">Target Base: ")

        elif option == 2:
            pass

        elif option == 0:
            break

        else:
            print("Invalid option!")
            continue
