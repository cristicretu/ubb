#
# This module is used to invoke the program's UI and start it. It should not contain a lot of code.
#
import test_program as tests
import ui

if __name__ == '__main__':
    tests.test_all_functions()
    ui.print_ui()
