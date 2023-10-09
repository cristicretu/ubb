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

z = create_z(1, 2)
print(to_str(z))




# fn that implement program requirements
