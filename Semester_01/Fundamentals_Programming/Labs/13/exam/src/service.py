class StudentException(Exception):
    def __init__(self, msg) -> None:
        super().__init__(msg)

class StudentService:
    def __init__(self, student_repository) -> None:
        self._student_repository = student_repository

    def check_valid_id(self, id):
        if id < 0 or id in [s.id for s in self._student_repository.get_all()]:
            raise StudentException("Invalid id!")

    def check_valid_name(self, name):
        if len(name) == 0:
            raise StudentException("Invalid name!")
        
        words = name.split(" ")
        if len(words) != 2:
            raise StudentException("Invalid name!")
        
    def check_valid_attendence(self, attendance):
        if attendance < 0 or int(attendance) != attendance:
            raise StudentException("Invalid attendance!")
        
    def check_valid_grade(self, grade):
        if grade < 0 or grade > 10:
            raise StudentException("Invalid grade!")

    def get_all_students(self):
        return sorted(self._student_repository.get_all(), 
                  key=lambda student: (-student.grade, student.name))
    
    def add_student(self, student):
        for s in self._student_repository.get_all():
            if s.id == student.id:
                raise StudentException("Student id already exists!")

        self._student_repository.add_student(student)


    def give_bonus(self, p, b):
        self._student_repository.give_bonus(p, b)

    def display_student_by_name(self, name):
        return sorted(self._student_repository.display_student_by_name(name), key=lambda student: student.name)