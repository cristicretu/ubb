from repository import GradeRepository, RepositoryError, StudentRepository


class StudentService:
    def __init__(self, filename="students.txt"):
        self.student_repository = StudentRepository(filename)

    def get_students(self):
        return self.student_repository.get_students()
    
    def get_student_by_id(self, id):
        if not isinstance(id, int):
            raise RepositoryError("Id must be an integer")

    def add_student(self, id, name, group):
        if not isinstance(id, int):
            raise RepositoryError("Id must be an integer")

        students = self.get_students()

        for stud in students:
            if stud.id == id:
                raise RepositoryError("Student with id already exists")

        self.student_repository.add_student(id, name, group)

        # undo redo here

    def remove_student_by_id(self, id):
        if not isinstance(id, int):
            raise RepositoryError("Id must be an integer")
        
        students = self.get_students()

        for stud in students:
            if stud.id == id:
                self.student_repository.remove_student_by_id(id)
                return
            
        raise RepositoryError("Student with id does not exist")

class GradeService:
    def __init__(self, filename="grades.txt"):
        self.grade_repository = GradeRepository(filename)

    def get_grades(self):
        return self.grade_repository.get_grades()
    
    def get_grade_by_id(self, id):
        if not isinstance(id, int):
            raise RepositoryError("Id must be an integer")
        
        grades = self.get_grades()

        for grade in grades:
            if grade.studentId == id:
                return grade
            
        raise RepositoryError("Grade with id does not exist")
    
    def assign_problem(self, studentId, laboratoryNo, laboratoryProblem, value=0):
        if not isinstance(studentId, int) or not isinstance(laboratoryNo, int) or not isinstance(laboratoryProblem, int) or not isinstance(value, int):
            raise RepositoryError("Id, number, problem and value must be integers")
        
        grades = self.get_grades()

        for grade in grades:
            if grade.studentId == studentId and grade.laboratoryNo == laboratoryNo:
                raise RepositoryError("Grade already exists")

        self.grade_repository.assign_problem(studentId, laboratoryNo, laboratoryProblem, value)
