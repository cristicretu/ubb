class Expense:
    def __init__(self, day, amount, type):
        self.__day = day
        self.__amount = amount
        self.__type = type

    @property
    def day(self):
        return self.__day

    @property
    def amount(self):
        return self.__amount

    @property
    def type(self):
        return self.__type

    def __str__(self):
        return f"Day: {self.__day} | Amount: {self.__amount} | Type: {self.__type}"

    def __eq__(self, other):
        return (
            self.__day == other.day
            and self.__amount == other.amount
            and self.__type == other.type
        )

    def to_dict(self):
        """
        Converts the expense to a dictionary.
        """
        return {"day": self.__day, "amount": self.__amount, "type": self.__type}

    @staticmethod
    def from_dict(data):
        """
        Converts a dictionary to an expense.
        """
        return Expense(data["day"], data["amount"], data["type"])
