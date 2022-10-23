############################################
# Author:  Zephyr Serret Verbist
# Date:    2020-10-20
# Version: 1.0
# ------------------------------------------
# Description:
#   This is the main file for the first question of the ARO course work 1.
#   It is a GUI that allows the user to change the parameters of the robot
#   and see the result in real time.
# Structure:
#   The GUI is defined in the __init__ function.
#   The canvas is updated every time the user changes a parameter (sliders). (see function update)
#   The axis are drawn in the function drawAxis.
#   The transformation matices of the links are calculated in the function linkTransformMat.
############################################

import tkinter as tk
import numpy as np

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Question 1")
        self.geometry("400x600")
        self.MyGrid = tk.Frame(self)
        self.MyGrid.grid(row=0, column=0, sticky="nsew")
        self.canvas = tk.Canvas(self.MyGrid, width=400, height=400, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=5)
        tk.Label(self.MyGrid, text="Link 1:").grid(row=1, column=0)
        tk.Label(self.MyGrid, text="length").grid(row=1, column=1)
        self.sliderl1 = tk.Scale(self.MyGrid, from_=80, to=180, orient=tk.HORIZONTAL,command=self.update)
        self.sliderl1.grid(row=1, column=2)
        tk.Label(self.MyGrid, text="angle").grid(row=1, column=3)
        self.slidert1 = tk.Scale(self.MyGrid, from_=-90, to=90, orient=tk.HORIZONTAL, command=self.update)
        self.slidert1.grid(row=1, column=4)
        tk.Label(self.MyGrid, text="Link 2:").grid(row=2, column=0)
        tk.Label(self.MyGrid, text="length").grid(row=2, column=1)
        self.sliderl2 = tk.Scale(self.MyGrid, from_=80, to=180, orient=tk.HORIZONTAL, command=self.update)
        self.sliderl2.grid(row=2, column=2)
        tk.Label(self.MyGrid, text="angle").grid(row=2, column=3)
        self.slidert2 = tk.Scale(self.MyGrid, from_=-90, to=90, orient=tk.HORIZONTAL, command=self.update)
        self.slidert2.grid(row=2, column=4)
        self.label = tk.Label(self.MyGrid, text="end effector coords : x: 0, y: 0")
        self.label.grid(row=3, column=0, columnspan=5)

        self.canvasToWorldSpace = np.matrix([[1, 0, 200],
                                             [0, -1, 350],
                                             [0, 0, 1]]) # move the origin to the base of the robot and flip the y axis
    def linkTransformMat(self, length, angle):
        rot =  np.matrix([[np.cos(angle), -np.sin(angle), 0],
                          [np.sin(angle), np.cos(angle), 0],
                          [0, 0, 1]])
        transpose = np.matrix([[1, 0, 0],
                               [0, 1, length],
                               [0, 0, 1]])
        return rot * transpose

    def drawAxis(self, transformMatrix, scale = 10, label=""):
        # draws the arrows of the axis according to the transform matrix
        axisBase = transformMatrix @ np.array([0,0,1])
        axisEnd1 = transformMatrix @ np.array([0,4 * scale,1])
        arrowEnd11 = transformMatrix @ np.array([1 * scale, 3 * scale, 1])
        arrowEnd12 = transformMatrix @ np.array([-1 * scale, 3 * scale, 1])
        axisEnd2 = transformMatrix @ np.array([4 * scale,0,1])
        arrowEnd21 = transformMatrix @ np.array([3 * scale, 1 * scale, 1])
        arrowEnd22 = transformMatrix @ np.array([3 * scale, -1 * scale, 1])

        self.canvas.create_line(axisBase[0,0], axisBase[0,1], axisEnd1[0,0], axisEnd1[0,1], fill="red", width=3)
        self.canvas.create_line(axisEnd1[0,0], axisEnd1[0,1], arrowEnd11[0,0], arrowEnd11[0,1], fill="red", width=3)
        self.canvas.create_line(axisEnd1[0,0], axisEnd1[0,1], arrowEnd12[0,0], arrowEnd12[0,1], fill="red", width=3)
        self.canvas.create_line(axisBase[0,0], axisBase[0,1], axisEnd2[0,0], axisEnd2[0,1], fill="blue", width=3)
        self.canvas.create_line(axisEnd2[0,0], axisEnd2[0,1], arrowEnd21[0,0], arrowEnd21[0,1], fill="blue", width=3)
        self.canvas.create_line(axisEnd2[0,0], axisEnd2[0,1], arrowEnd22[0,0], arrowEnd22[0,1], fill="blue", width=3)
        self.canvas.create_text(axisEnd1[0,0]-5*len(label)/2- 15, axisEnd1[0,1], text=label, fill="red")


    def update(self, event):
        self.canvas.delete("all")
        # Draw the base
        self.canvas.create_rectangle(20, 350, 380, 380, fill="grey")
        # Draw the first link
        origin = self.canvasToWorldSpace @ np.array([0,0,1])
        link1Transform = self.linkTransformMat(self.sliderl1.get(), np.deg2rad(self.slidert1.get()))
        endLink1 = self.canvasToWorldSpace @ link1Transform @ np.array([0,0,1])
        self.canvas.create_line(origin[0,0], origin[0,1], endLink1[0,0], endLink1[0,1], fill="black", width=5)
        # Draw first axis
        self.drawAxis(self.canvasToWorldSpace, label="World Space")
        # Draw the second link
        link2Transform = self.linkTransformMat(self.sliderl2.get(), np.deg2rad(self.slidert2.get()))
        worldEndLink2 = link1Transform @ link2Transform  @ np.array([0,0,1])
        endLink2 = self.canvasToWorldSpace @ link1Transform @ link2Transform @ np.array([0,0,1])
        self.canvas.create_line(endLink1[0,0], endLink1[0,1], endLink2[0,0], endLink2[0,1], fill="black", width=5)
        # Draw first's link axis
        self.drawAxis(self.canvasToWorldSpace @ link1Transform, label="Link1 effector")
        # Draw the second link axis (end effector)
        self.drawAxis(self.canvasToWorldSpace @ link1Transform @ link2Transform, label="End Effector")
        # Update the label

        self.label.config(text=f"end effector coords : x: {worldEndLink2[0,0]:.2f}, y: {worldEndLink2[0,1]:.2f}")



def Q1launch():
    app = App()
    app.update(None)
    app.mainloop()

if __name__ == "__main__":
    Q1launch()

