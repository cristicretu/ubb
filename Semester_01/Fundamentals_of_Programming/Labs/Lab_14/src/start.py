import sys
from ui.gui import GUI
from ui.ui import UI



def main(ui: str):
    if ui == "gui":
        gui = GUI()
        gui.run()
        return

    ui = UI()
    ui.run()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        user_interface_type = sys.argv[1]

        if user_interface_type == "--gui":
            main("gui")
        else:
            main("console")
    else:
        main("console")