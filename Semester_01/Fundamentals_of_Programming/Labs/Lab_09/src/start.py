import sys
from ui.ui import UI


if __name__ == "__main__":
    try:
        user_interface_mode = sys.argv[1]
    except IndexError:
        user_interface_mode = "console"

    user_interface = UI()
    user_interface._main_loop()
