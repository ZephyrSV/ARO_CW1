# using tktinter make a GUI with 4 buttons and a label

import tkinter as tk

from Q1 import Q1launch
from Q3 import Q3launch
from Q4 import Q4launch


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ARO course work 1")
        self.geometry("400x200")

        self.myGrid = tk.Frame(self)
        self.myGrid.grid(row=0, column=0, sticky="nsew")
        # Title
        self.label = tk.Label(self.myGrid, text="ARO course work 1", font=("Arial", 20, "bold"))
        self.label.grid(row=0, column=0, columnspan=2, sticky="nsew")
        # Button array
        self.label1 = tk.Label(self.myGrid, text="Coordinate Transform and FK")
        self.label1.grid(row=1, column=0, sticky="nsew")
        self.button1 = tk.Button(self.myGrid, text="Visualize Q1", command=self.button1)
        self.button1.grid(row=1, column=1, sticky="nsew")
        self.label2 = tk.Label(self.myGrid, text="Inverse Kinematics")
        self.label2.grid(row=2, column=0, sticky="nsew")
        self.button2 = tk.Button(self.myGrid, text="Visualize Q3", command=self.button2)
        self.button2.grid(row=2, column=1, sticky="nsew")
        self.button4 = tk.Button(self.myGrid, text="Visualize Q4", command=self.button4)
        self.button4.grid(row=3, column=1, sticky="nsew")
        self.label3 = tk.Label(self.myGrid, text="Motion Planning")
        self.label3.grid(row=4, column=0, sticky="nsew")
        self.button3 = tk.Button(self.myGrid, text="Visualize Q3", command=self.button3)
        self.button3.grid(row=4, column=1, sticky="nsew")

        self.myGrid.pack()
        # Footer
        self.label = tk.Label(self, text="This program was written by Zephyr Serret Verbist", font=("Arial", 10, "bold"))
        self.label.pack(side="bottom")

    def button1(self):
        Q1launch()

    def button2(self):
        Q3launch()

    def button3(self):
        self.label.config(text="Button 3 was pressed")

    def button4(self):
        Q4launch()

if __name__ == "__main__":
    app = App()
    app.mainloop()
