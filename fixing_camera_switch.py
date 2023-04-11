from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QMainWindow, QApplication, QLabel, QVBoxLayout, QPushButton, QGridLayout, QComboBox
from threading import Thread
from collections import deque
from datetime import datetime
import time
import sys
import cv2
import imutils
import numpy as np
from functools import partial
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QLineEdit
from PyQt5 import QtCore
import time
# from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt

# This code starts 2 cameras on separate threads to prevent frame lag
class CameraWidget(QtWidgets.QWidget):
    """Independent camera feed
    Uses threading to grab camera frames in the background
    """
    def __init__(self, width, height, data_table, stream_link=0, aspect_ratio=False, parent=None, deque_size=1):
        super(CameraWidget, self).__init__(parent)
        
        # Initialize deque used to store frames read from the stream
        self.deque = deque(maxlen=deque_size)
        self.positions = {}
        self.data_table = data_table
        # Slight offset is needed since PyQt layouts have a built in padding
        # So add offset to counter the padding 
        self.offset = 16
        self.screen_width = width - self.offset
        self.screen_height = height - self.offset
        self.maintain_aspect_ratio = aspect_ratio

        self.camera_stream_link = stream_link

        # Flag to check if camera is valid/working
        self.online = False
        self.capture = None
        self.video_frame = QtWidgets.QLabel()

        self.load_network_stream()
        
        # Start background frame grabbing
        self.get_frame_thread = Thread(target=self.get_frame, args=())
        self.get_frame_thread.daemon = True
        self.get_frame_thread.start()

        # Periodically set video frame to display
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.set_frame)
        self.timer.start(1)

        self.clicked = 0
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0
        self.x = 0
        self.y = 0
        self.magButton = False
        self.distance = 0

        print('Started camera: {}'.format(self.camera_stream_link))

    def load_network_stream_thread(self):
        if self.capture is not None:
            self.capture.release()

        self.capture = cv2.VideoCapture(self.camera_stream_link)

        if self.capture.isOpened():
            self.online = True
            self.camera_id = self.camera_stream_link
        else:
            self.online = False
            self.capture = None

    def load_network_stream(self):
        """Verifies stream link and open new stream if valid"""
        self.load_stream_thread = Thread(target=self.load_network_stream_thread, args=())
        self.load_stream_thread.daemon = True
        self.load_stream_thread.start()

    def get_frame(self):
        """Reads frame, resizes, and converts image to pixmap"""

        while True:
            if self.capture.isOpened() and self.online:
                # Read next frame from stream and insert into deque
                status, frame = self.capture.read()
                if status:
                    self.deque.append(frame)
                else:
                    self.capture.release()
                    self.online = False       



    def set_frame(self):
        """Sets pixmap image to video frame"""
        if self.deque and self.online:
            # Grab latest frame
            frame = self.deque[-1]

            # Keep frame aspect ratio
            if self.maintain_aspect_ratio:
                self.frame = imutils.resize(frame, width=self.screen_width)
            # Force resize
            else:
                self.frame = cv2.resize(frame, (self.screen_width, self.screen_height))

            # Convert to pixmap and set to video frame
            self.img = QtGui.QImage(self.frame, self.frame.shape[1], self.frame.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
            self.pix = QtGui.QPixmap.fromImage(self.img)
            self.video_frame.setPixmap(self.pix)

    def get_video_frame(self):
        return self.video_frame
    
    
    def close(self):
        # Release the camera and stop the frame grabbing thread
        print("i ran!")
        if self.capture is not None:
            self.capture.release()
        self.get_frame_thread.join()
        print("i finished!")



class App(QMainWindow):
    def __init__(self):
        super().__init__()


        self.arduino = None

        self.setWindowTitle("Magneto-Acoustic Control GUI")

        self.camera0ComboBox = QComboBox()
        self.camera1ComboBox = QComboBox()

        self.availableCameras = self.returnCameraIndexes()
        print("Avaiable Cameras")
        print(self.availableCameras)

        self.camera0ComboBox.addItem("Select Camera")
        self.camera1ComboBox.addItem("Select Camera")

        for i in self.availableCameras:
            self.camera0ComboBox.addItem("Camera " + str(i))
            self.camera1ComboBox.addItem("Camera " + str(i))

        self.camera0ComboBox.currentIndexChanged.connect(self.camera0Change)
        self.camera1ComboBox.currentIndexChanged.connect(self.camera1Change)

        com_ports = serial.tools.list_ports.comports()
        com_ports_names = [port.device for port in com_ports]
        self.com_port_box = QComboBox()
        self.com_port_box.addItems(com_ports_names)
        self.com_port_box.currentIndexChanged.connect(self.connect_to_arduino)
        

        self.x1_input = QLineEdit()
        self.x2_input = QLineEdit()
        self.y1_input = QLineEdit()
        self.y2_input = QLineEdit()
        self.z1_input = QLineEdit()
        self.z2_input = QLineEdit()

        self.x1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.x2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.y1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.y2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.z1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.z2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)

        # Create buttons to control magnetics and acoustics

        magneticButton = QPushButton('Turn Off Coils', self)
        magneticButton.setToolTip('Turn Off Coils')
        magneticButton.setStyleSheet('QPushButton {color: red}')
        magneticButton.clicked.connect(self.coilsOffClick)

        # Create Calibration button
        self.calibrationButton = QPushButton('Start Calibration', self)
        self.calibrationButton.setToolTip('Calibration Button')

        acousticPosButton = QPushButton('+', self)
        acousticPosButton.setToolTip('Increase Acoustic Field Intensity')
        acousticPosButton.clicked.connect(partial(self.acousticButtonPress, "U\n"))

        acousticNegButton = QPushButton('-', self)
        acousticNegButton.setToolTip('Decrease Acoustic Field Intensity')
        acousticNegButton.clicked.connect(partial(self.acousticButtonPress, "D\n"))

        acousticButton = QPushButton('Acoustics On/Off', self)
        acousticButton.setToolTip('Turn on/off acoustics')
        acousticButton.clicked.connect(partial(self.acousticButtonPress, "E\n"))
        acousticButton.setStyleSheet('QPushButton {color: red}')
        acousticButton.clicked.connect(partial(self.onAcousticClick, acousticButton))

        acousticReset = QPushButton('Acoustic Reset', self)
        acousticReset.setToolTip('Reset Acoustic Intensity to 0')
        acousticReset.clicked.connect(partial(self.acousticButtonPress, "R\n"))

        # Creating the labels for the buttons so direction is clear for user
        x1Label = QLabel(self)
        x1Label.setText("x1")
        x1Label.setAlignment(QtCore.Qt.AlignCenter)
        y1Label = QLabel(self)
        y1Label.setText("y1")
        y1Label.setAlignment(QtCore.Qt.AlignCenter)
        z1Label = QLabel(self)
        z1Label.setText("z1")
        z1Label.setAlignment(QtCore.Qt.AlignCenter)
        x2Label = QLabel(self)
        x2Label.setText("x2")
        x2Label.setAlignment(QtCore.Qt.AlignCenter)
        y2Label = QLabel(self)
        y2Label.setText("y2")
        y2Label.setAlignment(QtCore.Qt.AlignCenter)
        z2Label = QLabel(self)
        z2Label.setText("z2")
        z2Label.setAlignment(QtCore.Qt.AlignCenter)
        acousticLabel = QLabel(self)
        acousticLabel.setText("Acoustic Phase")
        acousticLabel.setAlignment(QtCore.Qt.AlignCenter)
        camera0Label = QLabel(self)
        camera0Label.setText("Camera 1 View:")
        camera1Label = QLabel(self)
        camera1Label.setText("Camera 2 View:")

        # magnificationInput = QLineEdit(self)
        magnificationLabel = QLabel(self)
        magnificationLabel.setText("Magnification:")

        # Add buttons to a gridlayout within the 2nd column of the main grid
        # Layout is nx3 
        button_grid = QGridLayout()
        button_grid.addWidget(self.x1_slider,0,0)
        button_grid.addWidget(x1Label,0,1)
        button_grid.addWidget(self.x1_input,0,2)

        button_grid.addWidget(self.x2_slider,1,0)
        button_grid.addWidget(x2Label,1,1)
        button_grid.addWidget(self.x2_input,1,2)

        button_grid.addWidget(self.y1_slider,2,0)
        button_grid.addWidget(y1Label,2,1)
        button_grid.addWidget(self.y1_input,2,2)

        button_grid.addWidget(self.y2_slider,3,0)
        button_grid.addWidget(y2Label,3,1)
        button_grid.addWidget(self.y2_input,3,2)

        button_grid.addWidget(self.z1_slider,4,0)
        button_grid.addWidget(z1Label,4,1)
        button_grid.addWidget(self.z1_input,4,2)

        button_grid.addWidget(self.z2_slider,5,0)
        button_grid.addWidget(z2Label,5,1)
        button_grid.addWidget(self.z2_input,5,2)
        
        button_grid.addWidget(acousticButton,6,0)
        button_grid.addWidget(acousticReset, 6, 2)

        button_grid.addWidget(acousticPosButton, 7, 0)
        button_grid.addWidget(acousticLabel, 7, 1)
        button_grid.addWidget(acousticNegButton, 7, 2)

        button_grid.addWidget(magnificationLabel, 8, 0)
        button_grid.addWidget(self.calibrationButton, 8, 1)
        button_grid.addWidget(magneticButton, 8, 2)

        button_grid.addWidget(camera0Label, 9, 0)
        button_grid.addWidget(camera1Label, 9, 1)

        button_grid.addWidget(self.camera0ComboBox, 10, 0)
        button_grid.addWidget(self.camera1ComboBox, 10, 1)
        button_grid.addWidget(self.com_port_box, 10, 2)
        
        
        # Create a table widget
        self.table = QtWidgets.QTableWidget()
        self.table.setRowCount(10)
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Index","X Pos", "Y Pos"])

        cw = QtWidgets.QWidget()
        self.my_grid = QtWidgets.QGridLayout()
        cw.setLayout(self.my_grid)
        self.setCentralWidget(cw)
        
        # Dynamically determine screen width/height
        self.screen_width = QtWidgets.QApplication.desktop().screenGeometry().width()
        self.screen_height = QtWidgets.QApplication.desktop().screenGeometry().height()
        
        # Stream links
        self.camera0 = None
        self.camera1 = None

        # Create camera widgets
        self.zero = None
        self.one = None

        print('Adding widgets to layout...')
        if self.zero is not None:
            self.my_grid.addWidget(self.zero.get_video_frame(),0,0,1,2)
        if self.one is not None:
            print("self one is not none")
            self.my_grid.addWidget(self.one.get_video_frame(),1,0,1,1)
        self.my_grid.addWidget(self.table,0,2,1,3)
        self.my_grid.addLayout(button_grid,0,5,1,2)
        
        print('Verifying camera work correctly')
    def connect_to_arduino(self, index):
        # Disconnect any previously opened serial port
        if self.arduino is not None and self.arduino.is_open:
            self.arduino.close()
        
        # Connect to selected serial port
        com_port = self.com_port_box.currentText()
        self.arduino = serial.Serial(com_port, 9600)
        self.setup_coils(self.arduino)
        self.setup_sliders()
        self.setup_inputs()

    def setup_sliders(self):
        self.x1_slider.sliderReleased.connect(partial(self.change_slider, self.coil1, self.x1_input, self.x1_slider))
        self.x2_slider.sliderReleased.connect(partial(self.change_slider, self.coil2, self.x2_input, self.x2_slider))
        self.y1_slider.sliderReleased.connect(partial(self.change_slider, self.coil3, self.y1_input, self.y1_slider))
        self.y2_slider.sliderReleased.connect(partial(self.change_slider, self.coil4, self.y2_input, self.y2_slider))
        self.z1_slider.sliderReleased.connect(partial(self.change_slider, self.coil5, self.z1_input, self.z1_slider))
        self.z2_slider.sliderReleased.connect(partial(self.change_slider, self.coil6, self.z2_input, self.z2_slider))

        sliders = [self.x1_slider, self.x2_slider, self.y1_slider, self.y2_slider, self.z1_slider, self.z2_slider]
        for slider in sliders:
            slider.setMaximum(255)
            slider.setMinimum(-255)

    def setup_inputs(self):
        self.x1_input.setFixedWidth(50)
        self.x1_input.returnPressed.connect(partial(self.set_slider, self.x1_slider, self.x1_input, self.coil1))
        self.x1_input.editingFinished.connect(partial(self.set_slider, self.x1_slider, self.x1_input, self.coil1))
        self.x2_input.setFixedWidth(50)
        self.x2_input.returnPressed.connect(partial(self.set_slider, self.x2_slider, self.x2_input, self.coil2))
        self.x2_input.editingFinished.connect(partial(self.set_slider, self.x2_slider, self.x2_input, self.coil2))
        self.y1_input.setFixedWidth(50)
        self.y1_input.returnPressed.connect(partial(self.set_slider, self.y1_slider, self.y1_input, self.coil3))
        self.y1_input.editingFinished.connect(partial(self.set_slider, self.y1_slider, self.y1_input, self.coil3))
        self.y2_input.setFixedWidth(50)
        self.y2_input.returnPressed.connect(partial(self.set_slider, self.y2_slider, self.y2_input, self.coil4))
        self.y2_input.editingFinished.connect(partial(self.set_slider, self.y2_slider, self.y2_input, self.coil4))
        self.z1_input.setFixedWidth(50)
        self.z1_input.returnPressed.connect(partial(self.set_slider, self.z1_slider, self.z1_input, self.coil5))
        self.z1_input.editingFinished.connect(partial(self.set_slider, self.z1_slider, self.z1_input, self.coil5))
        self.z2_input.setFixedWidth(50)
        self.z2_input.returnPressed.connect(partial(self.set_slider, self.z2_slider, self.z2_input, self.coil6))
        self.z2_input.editingFinished.connect(partial(self.set_slider, self.z2_slider, self.z2_input, self.coil6))
    
    def returnCameraIndexes(self):
        # checks the first 10 indexes.
        index = 0
        arr = []
        i = 10
        while i > 0:
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:
                print(index)
                arr.append(index)
            cap.release()
            index += 1
            i -= 1
        
        # if arr:
        #     arr.pop(-1)
        return arr

    def camera0Change(self, index):
        # print("Text changed:", s)
        print("View 1 was changed to " + str(index - 1))
        if self.camera0 is not None:
            self.zero.close()
            print("not none")
            cap = cv2.VideoCapture(index)
            print("cap index")
            cap.release()
            print("cap released")
            self.my_grid.removeWidget(self.zero.get_video_frame())
            print("widget removed")
            

        self.camera0 = self.availableCameras[index - 1]
        self.zero = CameraWidget(self.screen_width//3, self.screen_height//3, self.table, self.camera0)
        self.my_grid.addWidget(self.zero.get_video_frame(),0,0,1,2)
        self.calibrationButton.clicked.connect(partial(self.calibration_button, self.zero))

    def camera1Change(self, index):
        # print("Text changed:", s)
        print("View 2 was changed to " + str(index - 1))
        if self.camera1 is not None:
            self.my_grid.removeWidget(self.one.get_video_frame())
        self.camera1 = self.availableCameras[index - 1]
        self.one = CameraWidget(self.screen_width//3, self.screen_height//3, self.table, self.camera1)
        self.my_grid.addWidget(self.one.get_video_frame(),1,0,1,1)
    

    def calibration_button(self, camera):
        self.distance = 0
        if camera.magButton == True:
            camera.distance = ((camera.start_x-camera.end_x)**2-(camera.start_y-camera.end_y)**2)**(1/2)
            print(f"Distance: {camera.distance}, Magbutton: {camera.magButton}")
            camera.magButton = False
            camera.clicked = 0
            camera.start_x = 0
            camera.start_y = 0
            self.calibrationButton.setText("Start Calibration")
        else:
            camera.magButton = True
            self.calibrationButton.setText("End Calibration")


    def change_slider(self, coil, input, slider):
        value = slider.value()
        if (value < 0):
            coil.set_direction(0)
        else:
            coil.set_direction(1)
        print("value changed")

        coil.set_pwm(abs(value))
        input.setText(str(value))

    def set_slider(self, slider, input, coil):
        text = input.text()
        if text == "":
            value = 0
        elif text.startswith("-"):
            coil.set_direction(0)
            value_str = text[1:]
            if len(value_str) > 0:
                value = -int(value_str)
            else:
                value = 0
        else:
            coil.set_direction(1)
            value = int(text)
        slider.setValue(value)
        coil.set_pwm(abs(value))

    def acousticButtonPress(self, command):
        self.arduino.write(command.encode())
        print("clicked")
        time.sleep(2)
        text = self.arduino.readline()
        print(text)
    
    def setup_coils(self, arduino):
        m = [0.009874387,
            0.010136336,
            0.013625919,
            0.013743873,
            0.016740196,
            0.016591605]
        x = [-0.061568627,
            -0.063333333,
            -0.094705882,
            -0.082156863,
            -0.090980392,
            -0.090196078]
        r = [9.3, 8.9, 6.7, 6.8, 5.5, 5.5]


    # Turns the acoustic button red and green to represent off and on for
    # the user to understand the current state
    def onAcousticClick(self, button):
        if self.acousticOnOff == 0:
            self.acousticOnOff = 1 
            button.setStyleSheet('QPushButton {color: green}')
        else:
            self.acousticOnOff = 0 
            button.setStyleSheet('QPushButton {color: red}')
        # print("Acoustic Power = " + str(self.acousticOnOff)) # Debugging code

    def coilsOffClick(self):
        coils = [self.coil1, self.coil2, self.coil3, self.coil4, self.coil5, self.coil6]
        sliders = [self.x1_slider, self.x2_slider, self.y1_slider, self.y2_slider, self.z1_slider, self.z2_slider]
        inputs = [self.x1_input, self.x2_input, self.y1_input, self.y2_input, self.z1_input, self.z2_input]
        for i in range(len(coils)):
            coils[i].set_pwm(0)
            sliders[i].setValue(0)
            inputs[i].setText("0")
        print("Magnetics Off") # Debugging code

def exit_application():
    """Exit program event handler"""
    sys.exit(1)

if __name__ == '__main__':
    
    # Create main application window
    app = QApplication([])
    main_app = App()
    main_app.show()

    QtWidgets.QShortcut(QtGui.QKeySequence('Ctrl+Q'), main_app, exit_application)

    if(sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()