from random import randint


def check_if_valid_number(nr: int):
    fr = [0] * 10

    while nr:
        digit = nr % 10
        if fr[digit] > 0:
            raise ValueError("Invalid number")
        fr[digit] = 1
        nr //= 10

    return True


def generate_correct_randint():
    while True:
        try:
            n = randint(1000, 9999)
            assert check_if_valid_number(n)
            return n
        except:
            pass


def read_integer(prompt: str):
    while True:
        try:
            return int(input(prompt))
        except:
            print("Invalid integer, try again")
            continue


def count_codes_and_runners(target: int, nr: int):
    codes = 0
    runners = 0

    target_copy = target

    while target:
        target_digit = target % 10
        nr_digit = nr % 10

        if target_digit == nr_digit:
            codes += 1

        else:
            str_target = str(target_copy)
            str_digit = str(nr_digit)

            for chr in str_target:
                if str_digit == chr:
                    runners += 1
                    break

        target //= 10
        nr //= 10

    return codes, runners


if __name__ == "__main__":
    target_number = generate_correct_randint()
    while True:
        option = read_integer(">>>Enter a four digit number:")

        try:
            if option != 8086:
                assert check_if_valid_number(option)
        except:
            print("You Lose. The number you provided is invalid!")
            break

        if option == target_number:
            print("You win!")
        elif option == 8086:
            print("The number was" + str(target_number))
            break
        else:
            codes, runners = count_codes_and_runners(target_number, option)
            print(f"You have {codes} codes and {runners} runners")
