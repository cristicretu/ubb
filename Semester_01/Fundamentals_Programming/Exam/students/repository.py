from domain import Grade, Student


class RepositoryError(Exception):
    pass

class StudentRepository:
    def __init__(self, filename="students.txt"):
        self.filename = filename
        self.data = []
        self.__load__data()


    def __load__data(self):
        try:
            with open(self.filename, "r") as f:
                lines = f.read().splitlines()

                for line in lines:
                    id, name, group = line.split(",")
                    stud = Student(id, name, group)

                    self.data.append(stud)

        except IOError:
            raise RepositoryError("Failed to open file")

    def __save__data(self):
        try:
            with open(self.filename, "w") as f:
                for stud in self.data:
                    f.write(f"{stud.id},{stud.name},{stud.group}")

        except IOError:
            raise RepositoryError("Something bad happened")

    def get_students(self):
        return self.data
    
    def get_student_by_id(self, id):
        for stud in self.data:
            if stud.id == id:
                return stud
            
        return False

    def add_student(self, id, name, group):
        stud = Student(id, name, group)
        self.data.append(stud)

        self.__save__data()

    def remove_student_by_id(self, id):
        for stud in self.data:
            if stud.id == id:
                del stud

        self.__save__data()


class GradeRepository:
    def __init__(self, filename="grades.txt"):
        self.filename = filename
        self.data = []
        self.__load__data()


    def __load__data(self):
        try:
            with open(self.filename, "r") as f:
                lines = f.read().splitlines()

                for line in lines:
                    studentId, laboratoryNo, laboratoryProblem, value = line.split(",")
                    grade = Grade(studentId, laboratoryNo, laboratoryProblem, value)

                    self.data.append(grade)

        except IOError:
            raise RepositoryError("Failed to open file")

    def __save__data(self):
        try:
            with open(self.filename, "w") as f:
                for stud in self.data:
                    f.write(f"{stud.studentId},{stud.laboratoryNo},{stud.laboratoryProblem},{stud.value}")

        except IOError:
            raise RepositoryError("Something bad happened")
        
    def get_grades(self):
        return self.data
    
    def get_grade_by_student_id(self, id):
        for grade in self.data:
            if grade.studentId == id:
                return grade
            
        return False