# Received the window code from https://gist.github.com/docPhil99/ca4da12c9d6f29b9cea137b617c7b8b1

from PyQt5 import QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout, QPushButton
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import *
import sys
import cv2
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np
from functools import partial


class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self._run_flag = True

    def run(self):
        # capture from web cam
        # cap = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(2)
        while self._run_flag:
            # retval, img = cap.read()
            retval2, img2 = cap2.read()
            # if retval:
            #     self.change_pixmap_signal.emit(img)
            if retval2:
                self.change_pixmap_signal.emit(img2)
        # shut down capture system
        # cap.release()
        cap2.release()

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()



class App(QWidget):
    

    def __init__(self):

        
        super().__init__()

        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.mag = 0

        self.setWindowTitle("Qt live label demo")
        self.disply_width = 640
        self.display_height = 480
        # create the label that holds the image
        self.image_label = QLabel(self)
        self.image_label.resize(self.disply_width, self.display_height)
        # create a text label
        self.textLabel = QLabel('Webcam') # Not sure why it needs a webcam title, but crashes without

        ############ Trying to add buttons
        # Button to start video
        # self.ss_video = QtWidgets.QPushButton(self)
        # self.ss_video.setText('Start video')
        # self.ss_video.move(769, 100)
        # self.ss_video.resize(300, 100)

        xPosButton = QPushButton('+', self)
        xPosButton.setToolTip('Increase the x field')
        xPosButton.move(700,70)
        xPosButton.clicked.connect(partial(self.onCoilClick, "x", 1))
        # print(xPos) Debugging, for some reason it always resets to 0, but I was thinking it would
        # reinitialize to 0, but it's not repreinting xPos meaning it init only happens once
        # xPos = self.onClick(xPos, 1)
        # xPosButton.clicked.connect(self.onClick("x", 1))

        xNegButton = QPushButton('-', self)
        xNegButton.setToolTip('Decrease the x field')
        xNegButton.move(850,70)
        xNegButton.clicked.connect(partial(self.onCoilClick, "x", -1))

        yPosButton = QPushButton('+', self)
        yPosButton.setToolTip('Increase the y field')
        yPosButton.move(700,130)
        yPosButton.clicked.connect(partial(self.onCoilClick, "y", 1))

        yNegButton = QPushButton('-', self)
        yNegButton.setToolTip('Decrease the y field')
        yNegButton.move(850,130)
        yNegButton.clicked.connect(partial(self.onCoilClick, "y", -1))

        zPosButton = QPushButton('+', self)
        zPosButton.setToolTip('Increase the z field')
        zPosButton.move(700,190)
        zPosButton.clicked.connect(partial(self.onCoilClick, "z", 1))

        zNegButton = QPushButton('-', self)
        zNegButton.setToolTip('Decrease the z field')
        zNegButton.move(850,190)
        zNegButton.clicked.connect(partial(self.onCoilClick, "z", -1))

        acousticButton = QPushButton('Acoustics', self)
        acousticButton.setToolTip('Turn on/off acoustics')
        acousticButton.move(775,250)
        acousticButton.clicked.connect(partial(self.onMagClick))

        ################ Buttons



        # create a vertical box layout and add the two labels
        vbox = QVBoxLayout()
        vbox.addWidget(self.image_label)
        vbox.addWidget(self.textLabel)
        # set the vbox layout as the widgets layout
        self.setLayout(vbox)

        ################## Adding labels to the buttons
        xLabel = QLabel(self)
        xLabel.setText("x")
        vbox.addWidget(xLabel)

        # create the video capture thread
        self.thread = VideoThread()
        # connect its signal to the update_image slot
        self.thread.change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread.start()

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

    # when the direction buttons are clicked
    def onCoilClick(self, pos, value):
        # self.xPos = self.xPos + value
        # print('x changed value ' + str(self.xPos))
        # return self.xPos
        if (pos == "x"):
            # xPos = xPos + value
            # print('x changed value\n')
            self.xPos = self.xPos + value
            print('x changed value ' + str(self.xPos))
        elif (pos == "y"):
            self.yPos = self.yPos + value
            print('y changed value ' + str(self.yPos))
        else:
            self.zPos = self.zPos + value
            print('z changed value ' + str(self.zPos))

    def onMagClick(self):
        if self.mag == 0:
            self.mag = 1
        else:
            self.mag = 0
        print("Mag = " + str(self.mag))

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)
    
    def convert_cv_qt(self, cv_img):
        # Create a color range
        gray = np.array([180, 255, 50])
        black = np.array([0, 0, 0])

        """Convert from an opencv image to QPixmap"""
        # rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        mask = cv2.inRange(hsv, black, gray)

        dilated = dilatation(cv_img)
        hsvTest = cv2.cvtColor(dilated, cv2.COLOR_BGR2HSV)
        maskTest = cv2.inRange(hsvTest, black, gray)
        contours, hierarchy = cv2.findContours(maskTest, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for object in contours:
            objectArea = cv2.contourArea(object)
            if objectArea > 400:
                x, y, width, height = cv2.boundingRect(object)
                cv2.rectangle(cv_img, (x, y), (x + width, y + height), (255, 0, 0), 2)
                cv2.drawContours(cv_img, object, -1, (0, 0, 255), 2)
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
####################################### Extra Code

        # for 

#######################################

        convert_to_Qt_format = QtGui.QImage(cv_img.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

def dilatation(src):
        dilatation_size = 1
        dilation_shape = cv2.MORPH_ELLIPSE
        element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                        (dilatation_size, dilatation_size))
        dilatation_dst = cv2.dilate(src, element)
        # commented out the dilatation window and made it return the img instead
        return dilatation_dst
        # cv2.imshow("Dilatation", dilatation_dst)
    
if __name__=="__main__":
    app = QApplication(sys.argv)
    a = App()
    a.show()
    sys.exit(app.exec_())