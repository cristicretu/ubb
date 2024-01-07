class Bus:
    def __init__(self, id, code, model, times):
        self.id = id
        self.code = code
        self.model = model
        self.times = times

    def __str__(self):
        return "Bus: {}, {}, {}, {}".format(self.id, self.code, self.model, self.times)


class BusRoute:
    def __init__(self, code, length):
        self.code = code
        self.length = length