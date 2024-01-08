class Student:
    def __init__(self, id, name, attendance, grade):
        self.id = id
        self.name = name
        self.attendance = attendance
        self.grade = grade

    def __str__(self):
        return "Student: " + self.name + " " + str(self.id) + " " + str(self.attendance) + " " + str(self.grade)