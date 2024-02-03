class Student:
    def __init__(self, id, name, group):
        self.__id = id
        self.__name = name
        self.__group = group

    @property
    def id(self):
        return self.__id
    
    @property
    def name(self):
        return self.__name
    
    @property
    def group(self):
        return self.__group
    
    def __str__(self):
        return str(self.id) + " / " + str(self.name) + " / " + str(self.group)

class Grade:
    def __init__(self, studentId, laboratoryNo, laboratoryProblem, value):
        self.__studentId = studentId
        self.__laboratoryNo = laboratoryNo
        self.__laboratoryProblem = laboratoryProblem
        self.__value = value

    @property
    def studentId(self):
        return self.__studentId
    
    @property
    def laboratoryNo(self):
        return self.__laboratoryNo
    
    @property
    def laboratoryProblem(self):
        return self.__laboratoryProblem
    
    @property
    def value(self):
        return self.__value
    
    def __str__(self):
        return str(self.studentId) + " / " + str(self.laboratoryNo) + " / " + str(self.laboratoryProblem) + " / " + str(self.value)