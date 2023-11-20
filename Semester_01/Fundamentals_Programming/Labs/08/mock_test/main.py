# Functions Section
import re


def add_phone(phone_store: list, manufacturer: str, model: str, price: int) -> None:
    """
    Adds a new phone to the phone store list of dictonaries if the parameters pass the conditon:
    Len(parameter) >= 3

    :param phone_store: the list of phones
    :param manufacturer: a string representing the manufacturer of the phone
    :param model: the model of the phone of that manufacturer
    :param price: an integer representing the price of that model

    :return: the modified list with the new phone added of a string error message if the parameters do not meet the condition
    """
    if len(manufacturer) < 3 or len(model) < 3 or price < 100:
        return "Invalid parameters, each parameter should have at least 3 chars"

    phone_store.append({"manufacturer": manufacturer, "model": model, "price": price})

    return phone_store


def filter_phones_by_string(phone_store: list, manufacturer: str) -> list:
    """
    Filters the phone store list by the given manufacturer name, evne partially.

    :param phone_store: the list to be filtered
    :param manufacturer: the manufactuer that filters the list

    :return: the filtered list to be printed
    """
    filtered_phones = [
        phone for phone in phone_store if manufacturer in phone["manufacturer"]
    ]

    return filtered_phones


def increase_price_of_phone(
    phone_store: list, manufacturer: str, model: str, amount: int
):
    given_model = [
        phone
        for phone in phone_store
        if phone["manufacturer"] == manufacturer and phone["model"] == model
    ]

    if len(given_model) != 1:
        return "Invalid phone_store or model does not exist"

    given_model = given_model[0]
    given_model["price"] += amount

    return [
        phone
        if phone["model"] != model and phone["manufacturer"] != manufacturer
        else given_model
        for phone in phone_store
    ]


def increase_percent_of_phones(phone_store: list, percent: int):
    if percent > 100 or percent < -50:
        return "Invalid percent"

    return [
        {
            "manufacturer": phone["manufacturer"],
            "model": phone["model"],
            "price": int(phone["price"] * (1 + percent / 100)),
        }
        for phone in phone_store
    ]


# UI section


def read_str(prompt: str) -> int:
    while True:
        try:
            return str(input(prompt).strip())
        except:
            print("Invalid string, try again!")
            continue


def read_integer(prompt: str) -> int:
    while True:
        try:
            return int(input(prompt))
        except:
            print("Invalid integer, try again!")
            continue


def print_menu():
    print("1. Add a new phone: Enter manufacturer, model and price")
    print("2. Display all phones from a given manufacturer")
    print("3. Increase the price of a specific phone with a given amount.")
    print("4. Increase the price of all phones with a percent")
    print("0. Exit the application")


if __name__ == "__main__":
    phone_store = []

    ADD_PHONE = 1
    DISPLAY_PHONES = 2
    INCREASE_PRICE = 3
    MUL_PERCENT = 4
    EXIT = 0

    while True:
        print_menu()
        option = read_integer(">>> Option: ")

        if option == ADD_PHONE:
            manufacturer = read_str(">>> Manufacturer: ")
            model = read_str(">>> Model: ")
            price = read_integer(">>> Price:")

            add_phone(phone_store, manufacturer, model, price)
        elif option == DISPLAY_PHONES:
            manufacturer = read_str(">>> Manufacturer: ")

            filtered_phones = filter_phones_by_string(phone_store, manufacturer)

            print(filtered_phones)
        elif option == INCREASE_PRICE:
            manufacturer = read_str(">>> Manufacturer: ")
            model = read_str(">>> Model: ")
            amount = read_integer(">>> Amount: ")

            phone_store = increase_price_of_phone(
                phone_store, manufacturer, model, amount
            )
        elif option == MUL_PERCENT:
            percent = read_integer(">>> Percent: ")

            phone_store = increase_percent_of_phones(phone_store, percent)
        elif option == EXIT:
            print("Exiting!")
            break
        else:
            print("Option does not exist")
