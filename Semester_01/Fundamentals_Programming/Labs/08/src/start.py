import sys
import tkinter
from ui.gui import BookLibraryGUI
from ui.ui import UI


if __name__ == "__main__":
    try:
        user_interface_mode = sys.argv[1]
    except IndexError:
        user_interface_mode = "console"

    if user_interface_mode == "gui":
        root = tkinter.Tk()
        graphical_user_interface = BookLibraryGUI(root)
        root.mainloop()
    else:
        user_interface = UI()
        user_interface._main_loop()
