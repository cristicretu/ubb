class FunctionCall:
    def __init__(self, name_of_function, *parameters_of_function):
        self.__function_name = name_of_function
        self.__function_parameters = parameters_of_function

    def call(self):
        return self.__function_name(*self.__function_parameters)

    def __call__(self, *args, **kwargs):
        # overload the function call operator -- ()
        return self.call()


class Operation:
    def __init__(self, function_to_undo: FunctionCall, function_to_redo: FunctionCall):
        self.__function_undo = function_to_undo
        self.__function_redo = function_to_redo

    def undo(self):
        return self.__function_undo()

    def redo(self):
        return self.__function_redo()


class UndoError(Exception):
    def __init__(self, message):
        super().__init__(message)

    def __str__(self):
        return super().__str__()


class UndoService:
    def __init__(self):
        self.__history_of_operations = []
        self.__index_of_current_operation_in_history_list = 0

    def record(self, oper: Operation):
        self.__history_of_operations.append(oper)
        self.__index_of_current_operation_in_history_list += 1

    def undo(self):
        if self.__index_of_current_operation_in_history_list == 0:
            raise UndoError("No more undos")
        self.__index_of_current_operation_in_history_list -= 1
        self.__history_of_operations[
            self.__index_of_current_operation_in_history_list
        ].undo()

    def redo(self):
        if self.__index_of_current_operation_in_history_list >= len(
            self.__history_of_operations
        ):
            raise UndoError("No more redos")
        self.__history_of_operations[
            self.__index_of_current_operation_in_history_list
        ].redo()
        self.__index_of_current_operation_in_history_list += 1
