from domain import Student


class StudentRepository:
    def __init__(self, filename) -> None:
        self._students = []
        self._filename = filename
        self.load_data()    

    def load_data(self):
        try:
            input = open(self._filename, "r").read().split("\n")

            for line in input:
                if line == "":
                    continue

                id, name, attendance, grade = line.split(", ")
                student = Student(int(id), name, int(attendance), int(grade))
                self._students.append(student)
           
        except IOError as e:
            raise e
        

    def save_data(self):
        try:
            f = open(self._filename, "w")
            for student in self._students:
                f.write(str(student.id) + ", " + student.name + ", " + str(student.attendance) + ", " + str(student.grade) + "\n")
        except IOError as e:
            raise e
        finally:
            f.close()

    def add_student(self, student):
        self._students.append(student)

        self.save_data()

    def get_all(self):
        return self._students
    
    def give_bonus(self, p, b):
        for student in self._students:
            print(type(student.attendance), type(p))
            if student.attendance >= p:
                student.grade = min(10, student.grade + b)

        self.save_data()

    def display_student_by_name(self, name):
        students = []

        for student in self._students:
            if name in student.name:
                students.append(student)

        return students