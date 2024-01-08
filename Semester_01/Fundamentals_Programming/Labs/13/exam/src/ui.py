from domain import Student
from repository import StudentRepository
from service import StudentService


class UI:
    def __init__(self):
        self._student_service = StudentService(StudentRepository("students.txt"))

    def _print_menu(self):
        print("1. Add student")
        print("2. Display students")
        print("3. Give bonus")
        print("4. Display students by name")
        print("x. Exit")


    def run(self):
        while True:
            self._print_menu()

            option = input("Enter option: ")

            if option == "1":
                while True:
                    try:
                        id = int(input("Enter id: "))
                        self._student_service.check_valid_id(id)
                        break
                    except ValueError:
                        print("Invalid id!")
                    except Exception as ex:
                        print(ex)
                
                while True:
                    try:
                        name = input("Enter name: ")
                        self._student_service.check_valid_name(name)
                        break
                    except Exception as ex:
                        print(ex)
                
                while True:
                    try:
                        attendance = int(input("Enter attendance: "))
                        self._student_service.check_valid_attendence(attendance)
                        break
                    except ValueError:
                        print("Invalid attendance!")
                    except Exception as ex:
                        print(ex)

                while True:
                    try:
                        grade = int(input("Enter grade: "))
                        self._student_service.check_valid_grade(grade)
                        break
                    except ValueError:
                        print("Invalid grade!")
                    except Exception as ex:
                        print(ex)

                try:
                    stud = Student(id, name, attendance, grade)
                    self._student_service.add_student(stud)
                except Exception as ex:
                    print(ex)
            
            elif option == "2":
                students = self._student_service.get_all_students()

                for student in students:
                    print(student)

            elif option == "3":
                while True:
                    try:
                        attendance = int(input("Enter attendance: "))
                        self._student_service.check_valid_attendence(attendance)
                        break
                    except ValueError:
                        print("Invalid attendance!")
                    except Exception as ex:
                        print(ex)
                        
                while True:
                    try:
                        bonus = int(input("Enter grade: "))
                        self._student_service.check_valid_grade(bonus)
                        break
                    except ValueError:
                        print("Invalid bonus!")
                    except Exception as ex:
                        print(ex)

                self._student_service.give_bonus(attendance, bonus)

            elif option == "4":
                name = input("Enter name: ")

                students = self._student_service.display_student_by_name(name)

                for student in students:
                    print(student)

            elif option == "x":
                break

            else:
                print("Invalid option!")