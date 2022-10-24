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
#   The axes are drawn in the function drawAxis.
#   The transformation matrices of the links are calculated in the function linkTransformMat.
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
        return np.matrix([[np.cos(angle), -np.sin(angle), 0],
                          [np.sin(angle), np.cos(angle), length],
                          [0, 0, 1]])

    def drawAxis(self, transformMatrix, scale = 40, label="", offset = 0):
        # draws the arrows of the axis according to the transform matrix
        arrow = np.array([[0, 0, 1], # arrow origin
                          [0, 1, 1], # up arrow
                          [1, 0, 1]]) # right arrow
        scaleMatrix = np.matrix([[scale, 0, 0],
                                    [0, scale, 0],
                                    [0, 0, 1]])
        scaledArrow = scaleMatrix @ arrow.T
        transformedArrow = transformMatrix @ scaledArrow
        self.canvas.create_line(transformedArrow[0, 0], transformedArrow[1, 0], transformedArrow[0, 1], transformedArrow[1, 1], arrow=tk.LAST, fill="red", width=3)
        self.canvas.create_line(transformedArrow[0, 0], transformedArrow[1, 0], transformedArrow[0, 2], transformedArrow[1, 2], arrow=tk.LAST, fill="blue", width=3)
        self.canvas.create_text(transformedArrow[0,1]-5*len(label)/2- 15, transformedArrow[1,1]+offset, text=label, fill="red")



    def update(self, event):
        self.canvas.delete("all")
        # Draw the base
        self.canvas.create_rectangle(20, 350, 380, 380, fill="grey")
        # Get the transformation matrices of the links
        transMatWSL1 = self.linkTransformMat(0, np.deg2rad(self.slidert1.get()))
        transMatL1L2 = self.linkTransformMat(self.sliderl1.get(), np.deg2rad(self.slidert2.get()))
        transMatL2EE = self.linkTransformMat(self.sliderl2.get(), 0)
        # Get the transformation matrices of the links to canvas space
        transMatCanvasWS = self.canvasToWorldSpace
        transMatCanvasL1 = self.canvasToWorldSpace @ transMatWSL1
        transMatCanvasL2 = self.canvasToWorldSpace @ transMatWSL1 @ transMatL1L2
        transMatCanvasEE = self.canvasToWorldSpace @ transMatWSL1 @ transMatL1L2 @ transMatL2EE
        transMatWSEE = transMatWSL1 @ transMatL1L2 @ transMatL2EE

        # Draw the links
        origin = np.array([[0, 0, 1]]).T
        p1 = transMatCanvasL1 @ origin
        p2 = transMatCanvasL2 @ origin
        p3 = transMatCanvasEE @ origin
        p3WS = transMatWSEE @ origin

        self.canvas.create_line(p1[0, 0], p1[1, 0], p2[0, 0], p2[1, 0], fill="black", width=3)
        self.canvas.create_line(p2[0, 0], p2[1, 0], p3[0, 0], p3[1, 0], fill="black", width=3)
        # Draw the axes
        self.drawAxis(transMatCanvasWS, label="World Space", offset=20)
        self.drawAxis(transMatCanvasL1, label="L1")
        self.drawAxis(transMatCanvasL2, label="L2")
        self.drawAxis(transMatCanvasEE, label="EE")
        # Update the label
        self.label.config(text="end effector coords : x: {}, y: {}".format(int(p3WS[0, 0]), int(p3WS[1, 0])))




def Q1launch():
    app = App()
    app.update(None)
    app.mainloop()

if __name__ == "__main__":
    Q1launch()

