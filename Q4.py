############################################
# Author:  Zephyr Serret Verbist
# Date:    2020-10-20
# Version: 1.0
# ------------------------------------------
#   This is the main file for the fourth question of the ARO course work 1.
#   It is a GUI that allows the user to change the parameters of the robot
#   and to observe the changes to the jacobian matrix in real time.
#   The user can choose between the two different methods of calculating the jacobian matrix.
#   Additionally, the user can determine a target and use the GO button
#   to make the robot move to that target.
# Structure:
#   The GUI is defined in the __init__ function.
#   The canvas is updated every time the user changes a parameter (sliders). (see function update)
#   The Jacobian matrix is calculated in the function jacobian.
#   The transformation matrices of the links are calculated in the function linkTransformMat.
#   The button "GO" calls the function GO.
############################################

import tkinter as tk
import numpy as np


class Q4App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.link1Length = 80
        self.link2Length = 80
        self.jacobianUseL3L4 = False
        self.canvasToWorldSpace = np.matrix([[1, 0, 200],
                                             [0, -1, 350],
                                             [0, 0, 1]])  # move the origin to the base of the robot and flip the y axis
        self.title("Question 4")
        self.geometry("450x700")
        self.MyGrid = tk.Frame(self)
        self.MyGrid.grid(row=0, column=0, sticky="nsew")
        self.canvas  = tk.Canvas(self.MyGrid, width=450, height=400, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=5)
        tk.Label(self.MyGrid, text="Link 1 angle:").grid(row=1, column=0)
        #variable for the slider
        self.slider1 = tk.Scale(self.MyGrid, from_=0, to=180, orient=tk.HORIZONTAL, command=self.update)
        self.slider1.set(45)
        self.slider1.grid(row=1, column=1)
        tk.Label(self.MyGrid, text="Link 2 angle:").grid(row=1, column=2)
        self.slider2 = tk.Scale(self.MyGrid, from_=0, to=180, orient=tk.HORIZONTAL, command=self.update)
        self.slider2.set(135)
        self.slider2.grid(row=1, column=3)
        tk.Label(self.MyGrid, text="Link 3 angle:").grid(row=2, column=0)
        self.slider3 = tk.Scale(self.MyGrid, from_=0, to=180, orient=tk.HORIZONTAL, command=self.update)
        self.slider3.set(45)
        self.slider3.grid(row=2, column=1)
        tk.Label(self.MyGrid, text="Link 4 angle:").grid(row=2, column=2)
        self.slider4 = tk.Scale(self.MyGrid, from_=0, to=180, orient=tk.HORIZONTAL, command=self.update)
        self.slider4.set(135)
        self.slider4.grid(row=2, column=3)

        self.label = tk.Label(self.MyGrid, text="current end effector coords : x: 0, y: 0")
        self.label.grid(row=3, column=0, columnspan=4)
        self.label2 = tk.Label(self.MyGrid, text="target position :")
        self.label2.grid(row=4, column=0, columnspan=4)
        self.entry1 = tk.Entry(self.MyGrid)
        self.entry1.insert(0, "-43")
        self.entry1.grid(row=5, column=0, columnspan=2)
        self.entry2 = tk.Entry(self.MyGrid)
        self.entry2.insert(0, "136")
        self.entry2.grid(row=5, column=2, columnspan=2)
        self.checkButton = tk.Checkbutton(self.MyGrid, text="Use link3 and link4 in jacobian", command=self.jacobianToggle)
        self.checkButton.grid(row=6, column=0, columnspan=4)
        self.button = tk.Button(self.MyGrid, text="GO", command=self.GO)
        self.button.grid(row=7, column=0, columnspan=4, sticky="nsew", pady=10, padx=10)
        self.label3 = tk.Label(self.MyGrid, text="current jacobian matrix :")
        self.label3.grid(row=8, column=0, columnspan=4)
        self.label4 = tk.Label(self.MyGrid, text="jacobian")
        self.label4.grid(row=9, column=0, columnspan=4)

    def linkTransformMat(self, length, angle, x=0):
        return np.matrix([[np.cos(angle), -np.sin(angle), x],
                          [np.sin(angle), np.cos(angle), length],
                          [0, 0, 1]])


    def update(self, event):
        self.canvas.delete("all")
        # Draw the base
        self.canvas.create_rectangle(20, 350, 430, 380, fill="grey")
        # Get the transformation matrices of the links
        t1 = np.deg2rad(self.slider1.get()-90)
        t2 = np.deg2rad(self.slider2.get()-90)
        t3 = np.deg2rad(self.slider3.get()-90)
        t4 = np.deg2rad(self.slider4.get()-90)
        transMatWSL1 = self.linkTransformMat(0, t1, x=-100)
        transMatL1L2 = self.linkTransformMat(self.link1Length, t2)
        transMatL2EE = self.linkTransformMat(self.link2Length, 0)
        transMatWSL3 = self.linkTransformMat(0, t3, x=100)
        transMatL3L4 = self.linkTransformMat(self.link1Length, t4)
        transMatL4FF = self.linkTransformMat(self.link2Length, 0)
        # Get the transformation matrices of the links to canvas space
        transMatCanvasL1 = self.canvasToWorldSpace @ transMatWSL1
        transMatCanvasL2 = self.canvasToWorldSpace @ transMatWSL1 @ transMatL1L2
        transMatCanvasEE = self.canvasToWorldSpace @ transMatWSL1 @ transMatL1L2 @ transMatL2EE
        transMatCanvasL3 = self.canvasToWorldSpace @ transMatWSL3
        transMatCanvasL4 = self.canvasToWorldSpace @ transMatWSL3 @ transMatL3L4
        transMatCanvasFF = self.canvasToWorldSpace @ transMatWSL3 @ transMatL3L4 @ transMatL4FF
        transMatWSEE = transMatWSL1 @ transMatL1L2 @ transMatL2EE

        # Draw the links
        origin = np.array([[0, 0, 1]]).T
        p1 = transMatCanvasL1 @ origin
        p2 = transMatCanvasL2 @ origin
        p3 = transMatCanvasEE @ origin
        q1 = transMatCanvasL3 @ origin
        q2 = transMatCanvasL4 @ origin
        q3 = transMatCanvasFF @ origin
        p3WS = transMatWSEE @ origin
        self.canvas.create_line(p1[0, 0], p1[1, 0], p2[0, 0], p2[1, 0], fill="black", width=3)
        self.canvas.create_line(p2[0, 0], p2[1, 0], p3[0, 0], p3[1, 0], fill="gray", width=3)
        target = self.canvasToWorldSpace * np.array([[float(self.entry1.get()), float(self.entry2.get()), 1]]).T
        self.canvas.create_oval(target[0, 0] - 5, target[1, 0] - 5, target[0, 0] + 5, target[1, 0] + 5, fill="blue")
        self.canvas.create_oval(p3[0, 0] - 5, p3[1, 0] - 5, p3[0, 0] + 5, p3[1, 0] + 5, fill="red")
        self.canvas.create_line(q1[0, 0], q1[1, 0], q2[0, 0], q2[1, 0], fill="black", width=3)
        self.canvas.create_line(q2[0, 0], q2[1, 0], q3[0, 0], q3[1, 0], fill="gray", width=3)
        # Update the coord label
        self.label.config(text="end effector coords : x: {}, y: {}".format(int(p3WS[0, 0]), int(p3WS[1, 0])))
        #update the jacobian matrix
        self.label4.config(text="{}".format(self.jacobian(t1,t2,t3,t4)))


    def jacobian(self, t1, t2, t3, t4):
        # the transformation matrices of the links
        transMatWSL1 = self.linkTransformMat(0, t1, x=-100)
        transMatL1L2 = self.linkTransformMat(self.link1Length, t2)
        transMatL2EE = self.linkTransformMat(self.link2Length, 0)
        transMatWSL3 = self.linkTransformMat(0, t3, x=100)
        transMatL3L4 = self.linkTransformMat(self.link1Length, t4)
        # the transformation matrices of the links to World Space
        transMatWSL2 = transMatWSL1 @ transMatL1L2
        transMatWSL4 = transMatWSL3 @ transMatL3L4
        transMatWSEE = transMatWSL2 @ transMatL2EE
        # get the position of the end effector and links
        origin = np.array([[0, 0, 1]]).T
        EEPos = transMatWSEE @ origin
        L1Pos = transMatWSL1 @ origin
        L2Pos = transMatWSL2 @ origin
        L3Pos = transMatWSL3 @ origin
        L4Pos = transMatWSL4 @ origin
        # calculate the jacobian
        jacobian = np.vstack([np.cross([0,0,1], (EEPos-L1Pos).T), np.cross([0,0,1], (EEPos-L2Pos).T)])
        if self.jacobianUseL3L4: #allows us to switch between using L1 L2 L3 L4 or just L1 L2
            jacobian = np.vstack([jacobian, np.cross([0,0,1], (EEPos-L3Pos).T), np.cross([0,0,1], (EEPos-L4Pos).T)])
        else:
            jacobian = np.vstack([jacobian, np.zeros((1,3)), np.zeros((1,3))])
        return jacobian.T[0:2, :]


    def GO(self):
        t1 = np.deg2rad(self.slider1.get()-90)
        t2 = np.deg2rad(self.slider2.get()-90)
        t3 = np.deg2rad(self.slider3.get()-90)
        t4 = np.deg2rad(self.slider4.get()-90)
        jacobian = self.jacobian(t1, t2, t3, t4)
        # get the desired position
        target = np.array([int(self.entry1.get()), int(self.entry2.get())]).T
        # get the current position
        transMatWSL1 = self.linkTransformMat(0, np.deg2rad(self.slider1.get() - 90), x=-100)
        transMatL1L2 = self.linkTransformMat(self.link1Length, np.deg2rad(self.slider2.get() - 90))
        transMatL2EE = self.linkTransformMat(self.link2Length, 0)
        currPos = transMatWSL1 @ transMatL1L2 @ transMatL2EE @ np.array([[0, 0, 1]]).T
        currPos = np.array([currPos[0, 0], currPos[1, 0]]).T
        # calculate the error
        dy = target - currPos

        dt = np.linalg.pinv(jacobian) @ dy
        self.slider1.set(np.rad2deg(t1 + dt[0])+90)
        self.slider2.set(np.rad2deg(t2 + dt[1])+90)
        self.slider3.set(np.rad2deg(t3 + dt[2])+90)
        self.slider4.set(np.rad2deg(t4 + dt[3])+90)
        self.update(None)

    def jacobianToggle(self):
        self.jacobianUseL3L4 = not self.jacobianUseL3L4
        self.update(None)




def Q4launch():
    app = Q4App()
    app.update(None)
    app.mainloop()

if __name__ == "__main__":
    Q4launch()

