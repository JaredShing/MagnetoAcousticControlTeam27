from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QApplication, QLabel, QPushButton, QGridLayout, QComboBox
from threading import Thread
from collections import deque
import time
import sys
import cv2
from numpy import array as np_array
from functools import partial
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QLineEdit
from PyQt5 import QtCore
import time
import math
from datetime import datetime
# import matplotlib.pyplot as plt
# from matplotlib.figure import Figure
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import numpy as np

# This code starts 2 cameras on separate threads to prevent frame lag
class CameraWidget(QtWidgets.QWidget):
    """Independent camera feed
    Uses threading to grab camera frames in the background
    """
    def __init__(self, width, height, data_table, stream_link=0, parent=None, deque_size=1):
        super(CameraWidget, self).__init__(parent)
 
        # Initialize deque used to store frames read from the stream
        self.deque = deque(maxlen=deque_size)
        self.positions = {}
        self.data_table = data_table

        self.screen_width = width
        self.screen_height = height
        self.camera_stream_link = stream_link

        # Flag to check if camera is valid/working
        self.online = False
        self.capture = None
        self.video_frame = QtWidgets.QLabel()
        self.video_frame.mousePressEvent = self.mouse_callback
        self.video_frame.mouseMoveEvent = self.mouse_move_callback

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
        self.particles = {}
        self.error = 0
        self.tracking_enabled = False

        print('Started camera: {}'.format(self.camera_stream_link))

    def load_network_stream_thread(self):
        self.capture = cv2.VideoCapture(self.camera_stream_link)
        self.online = True

    def load_network_stream(self):
        """Verifies stream link and open new stream if valid"""

        self.load_stream_thread = Thread(target=self.load_network_stream_thread, args=())
        self.load_stream_thread.daemon = True
        self.load_stream_thread.start()


    def get_frame(self):
        """Reads frame, resizes, and converts image to pixmap"""

        while True:
            try:
                if self.capture.isOpened() and self.online:
                    # Read next frame from stream and insert into deque
                    status, frame = self.capture.read()
                    if status:
                        self.deque.append(frame)
                    else:
                        self.capture.release()
                        self.online = False
                self.spin(.001)                    

            except AttributeError:
                pass

    def spin(self, seconds):
        """Pause for set amount of seconds, replaces time.sleep so program doesnt stall"""

        time_end = time.time() + seconds
        while time.time() < time_end:
            QtWidgets.QApplication.processEvents()

    def set_frame(self):
        """Sets pixmap image to video frame"""

        if not self.online:
            self.spin(1)
            return

        if self.deque and self.online:
            # Grab latest frame
            frame = self.deque[-1]
            
            self.frame = cv2.resize(frame, (self.screen_width, self.screen_height))
            # Add green line to indicte clicked point
            
            if self.tracking_enabled:
                print("Tracking Enabled")
                self.add_contours()

            if self.magButton:
                if self.clicked == 1:
                    if self.x + self.y == 0:
                        self.x = self.start_x
                        self.y = self.start_y
                    cv2.line(self.frame, (self.start_x, self.start_y), (self.x, self.y), color=(0, 255, 0), thickness=2)
                if self.clicked == 2:
                    cv2.line(self.frame, (self.start_x, self.start_y), (self.end_x, self.end_y), color=(0, 255, 0), thickness=2)

            cv2.circle(self.frame, (int(self.screen_width/2), int(self.screen_height/2)),10,color=(0, 0, 255))
            
            # Was getting a complex number error if calibration button was hit before ending 2nd point
            if not isinstance(self.distance, complex):
                self.scale_text =  f"Scale: {round(self.distance/10,3)}px/mm"

            cv2.putText(frame, self.scale_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # cv2.putText(frame, str(round(self.error, 3)), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # print(self.particles)
            # Convert to pixmap and set to video frame
            self.img = QtGui.QImage(self.frame, self.frame.shape[1], self.frame.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
            self.pix = QtGui.QPixmap.fromImage(self.img)
            self.video_frame.setPixmap(self.pix)

    def get_video_frame(self):
        return self.video_frame
    
    def mouse_callback(self, event):
        difference = self.screen_height - self.video_frame.size().height()
        offset = int(difference/2)

        if self.magButton:
            self.clicked += 1
            if self.clicked == 2:
                self.end_x = event.pos().x()
                self.end_y = event.pos().y()+offset
                self.video_frame.setMouseTracking(False)
                print("Mouse clicked at x={}, y={}, clicked = {}".format(self.end_x, self.end_y, self.clicked))
            else:
                self.video_frame.setMouseTracking(True)
                self.start_x = event.pos().x()
                self.start_y = event.pos().y()+offset
                print("Mouse clicked at x={}, y={}, clicked = {}".format(self.start_x, self.start_y, self.clicked))

    def mouse_move_callback(self, event):
        y_difference = self.screen_height - self.video_frame.size().height()
        y_offset = int(y_difference/2)
        
        self.x = event.pos().x()
        self.y = event.pos().y()+y_offset

    def add_contours(self):
        # Create a color range
        gray = np.array([180, 255, 50])
        black = np.array([0, 0, 0])

        """Convert from an opencv image to QPixmap"""
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(hsv, black, gray)

        dilated = self.dilatation(self.frame)
        hsvTest = cv2.cvtColor(dilated, cv2.COLOR_BGR2HSV)
        maskTest = cv2.inRange(hsvTest, black, gray)
        contours, hierarchy = cv2.findContours(maskTest, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Keep only the most recent 5 particles
        self.particles = {i: particle for i, particle in self.particles.items() if len(particle['positions']) > 0}
        particle_ids = sorted(self.particles.keys(), reverse=True)[:5]
        self.particles = {i: self.particles[i] for i in particle_ids}

        self.current_particles = {}
        for i, contour in enumerate(contours):
            objectArea = cv2.contourArea(contour)
            if objectArea > 400:
                x, y, width, height = cv2.boundingRect(contour)
                com = [x+width/2, y+height/2] # get centroid
                self.current_particles[i] = (com[0], com[1])

        for i, (xc1, yc1) in self.current_particles.items():
            add_particle = True
            for j, particle in self.particles.items():
                if np.linalg.norm(np.array([xc1, yc1]) - np.array(particle['positions'][-1])) < 50:
                    add_particle = False
                    particle['positions'].append((xc1, yc1))
                    # self.error = np.linalg.norm(np.array(particle['positions'][-1])-np.array([self.screen_width/2, self.screen_height/2]))
            if add_particle:
                if len(self.particles) < 5:
                    # Add a new particle if there are less than 5 particles
                    self.particles[i] = {'positions': [(xc1, yc1)]}
                else:
                    # Replace the oldest particle with the new particle if there are already 5 particles
                    oldest_particle_id = min(self.particles.keys(), key=lambda x: len(self.particles[x]['positions']))
                    self.particles[oldest_particle_id] = {'positions': [(xc1, yc1)]}

            # Keep only the most recent 10 positions for each particle
            if i in self.particles:
                self.particles[i]['positions'] = self.particles[i]['positions'][-10:]

        self.update_table(self.data_table, self.particles)

        # Draw bounding boxes around the objects in the current frame
        

        # Draw bounding boxes around the objects in the current frame
        for i, particle in self.particles.items():
            # Get the most recent position of the particle
            x, y = particle['positions'][-1]
        

            # Draw the bounding box
            width = 20
            height = 20
            cv2.rectangle(self.frame, (int(x), int(y)), (int(x + width), int(y + height)), (255, 0, 0), 2)
            # Draw the particle number inside the bounding box
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.frame, str(i), (int(x), int(y)-5), font, 0.5, (0, 0, 255), 1)

            # Draw the line connecting the previous positions
            if len(particle['positions']) > 1:
                for j in range(len(particle['positions'])-1):
                    x1, y1 = particle['positions'][j]
                    x2, y2 = particle['positions'][j+1]
                    cv2.line(self.frame, (int(x1+width/2), int(y1+height/2)), (int(x2+width/2), int(y2+height/2)), (0, 255, 0), 2)


                
    def update_table(self, tableWidget, data):
    # Clear the table
        tableWidget.clearContents()
        # Add the updated dictionary data to the table
        for i, (index, value) in enumerate(data.items()):
            # Extract the most recent position from the array
            x, y = value['positions'][-1]
            tableWidget.setItem(i, 0, QtWidgets.QTableWidgetItem(str(index)))
            tableWidget.setItem(i, 1, QtWidgets.QTableWidgetItem(str(x)))
            tableWidget.setItem(i, 2, QtWidgets.QTableWidgetItem(str(y)))
            error = round(np.linalg.norm(np.array([x,y])-np.array([self.screen_width/2, self.screen_height/2])),3)
            tableWidget.setItem(i, 3, QtWidgets.QTableWidgetItem(str(error)))
                    
    def get_position_data(self):
        return self.positions
    
    def dilatation(self, src):
        dilatation_size = 1
        dilation_shape = cv2.MORPH_ELLIPSE
        element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                        (dilatation_size, dilatation_size))
        dilatation_dst = cv2.dilate(src, element)
        # commented out the dilatation window and made it return the img instead
        return dilatation_dst
    
    def close(self):
        # Release the camera and stop the frame grabbing thread
        print("i ran!")
        self.capture.release()
        self.get_frame_thread.join()

def exit_application():
    """Exit program event handler"""
    sys.exit(1)

class Coil():
    def __init__(self, axis, num, resistance, m, b, arduino):
        self.axis = axis
        self.num = num
        self.resistance = resistance
        self.voltage = 24
        self.pwm_value = 0
        self.PWM_MAX = 255
        self.current = 0
        self.direction = 1
        self.arduino = arduino
        self.m = m
        self.b = b
        self.calc_current()

    def set_pwm(self, value):
        string_to_send = f"{self.axis}{self.num}{str(value).zfill(3)}{self.direction}\n"
        try:
            self.arduino.write(string_to_send.encode())
        except serial.SerialException as e:
            print(f"Failed to write PWM to Arduino: ")
        print(string_to_send)
        time.sleep(0.1)

    # return the calculated current value based on the pwm value
    def get_current_value(self):
        return self.current

    # Set the direction of the coil
    def set_direction(self, direction_request):
        self.direction = direction_request
        time.sleep(0.1)

    # Calculate the current using the linear regression calculated experimentally
    def calc_current(self):
        self.current = self.pwm_value*self.m + self.b

class App(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Magneto-Acoustic Control GUI")

        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.acoustic = 0
        self.acousticOnOff = 0
        self.magneticsOnOff = 0
        self.arduino = None

        self.camera0ComboBox = QComboBox()
        self.camera1ComboBox = QComboBox()

        self.availableCameras = self.returnCameraIndexes()

        self.camera0ComboBox.addItem("Select Camera")
        self.camera1ComboBox.addItem("Select Camera")

        for i in self.availableCameras:
            self.camera0ComboBox.addItem("Camera " + str(i))
            self.camera1ComboBox.addItem("Camera " + str(i))

        self.camera0ComboBox.currentIndexChanged.connect(self.camera0Change)
        self.camera1ComboBox.currentIndexChanged.connect(self.camera1Change)

        # com_ports = serial.tools.list_ports.comports()
        # com_ports_names = [port.device for port in com_ports]
        # self.com_port_box = QComboBox()
        # self.com_port_box.addItems(com_ports_names)
        # self.com_port_box.currentIndexChanged.connect(self.connect_to_arduino)
        com_ports = serial.tools.list_ports.comports()
        com_port_info = [(port.description) for port in com_ports]
        self.com_port_box = QComboBox()
        self.com_port_box.setFixedWidth(200)  # Set the width to 200 pixels
        self.com_port_box.setSizeAdjustPolicy(QComboBox.AdjustToContents)  # Adjust the size of the drop-down to fit the content
        self.com_port_box.addItem("Select Com Port")
        for device_name in com_port_info:
            self.com_port_box.addItem(f"{device_name}")
        self.com_port_box.currentIndexChanged.connect(self.connect_to_arduino)

        x1_label = QLabel(self)
        x2_label = QLabel(self)
        y1_label = QLabel(self)
        y2_label = QLabel(self)
        z1_label = QLabel(self)
        z2_label = QLabel(self)
        m1_label = QLabel(self)
        m2_label = QLabel(self)
        m3_label = QLabel(self)
        m4_label = QLabel(self)

        self.x1_input = QLineEdit()
        self.x2_input = QLineEdit()
        self.y1_input = QLineEdit()
        self.y2_input = QLineEdit()
        self.z1_input = QLineEdit()
        self.z2_input = QLineEdit()
        self.m1_input = QLineEdit()
        self.m2_input = QLineEdit()
        self.m3_input = QLineEdit()
        self.m4_input = QLineEdit()

        self.up = QPushButton("UP")
        self.down = QPushButton("DOWN")
        self.left = QPushButton("LEFT")
        self.right = QPushButton("RIGHT")


        self.direction_buttons = []

        for i in range(10):
            self.direction_buttons.append(QPushButton("+"))


        self.x1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.x2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.y1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.y2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.z1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.z2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.m1_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.m2_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.m3_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.m4_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)


        # Create buttons to control magnetics and acoustics
        self.magneticButton = QPushButton('Turn Off Coils', self)
        self.magneticButton.setToolTip('Turn Off Coils')
        self.magneticButton.setStyleSheet('QPushButton {color: red}')

        # Create Calibration button
        self.calibrationButton = QPushButton('Start Calibration', self)
        self.calibrationButton.setToolTip('Calibration Button')

        self.acousticPosButton = QPushButton('+', self)
        self.acousticPosButton.setToolTip('Increase Acoustic Field Intensity')

        self.acousticNegButton = QPushButton('-', self)
        self.acousticNegButton.setToolTip('Decrease Acoustic Field Intensity')

        self.acousticButton = QPushButton('Acoustics On/Off', self)
        self.acousticButton.setToolTip('Turn on/off acoustics')
        self.acousticButton.setStyleSheet('QPushButton {color: red}')

        self.acousticReset = QPushButton('Acoustic Reset', self)
        self.acousticReset.setToolTip('Reset Acoustic Intensity to 0')

        debug_button = QPushButton('Debug', self)
        debug_button.clicked.connect(self.toggle_terminal)
        self.debug_terminal = QtWidgets.QPlainTextEdit(self)
        self.debug_terminal.setReadOnly(True)
        self.debug_terminal.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.debug_terminal.setVisible(False)

        # Creating the labels for the buttons so direction is clear for user
        text_labels = ["x1", "x2", "y1", "y2", "z1", "z2", "m1", "m2", "m3", "m4"]
        labels = [x1_label, x2_label, y1_label, y2_label, z1_label, z2_label, m1_label, m2_label, m3_label, m4_label]
        for i in range(len(text_labels)):
            labels[i].setText(text_labels[i])
            labels[i].setAlignment(QtCore.Qt.AlignCenter)

        acousticLabel = QLabel(self)
        acousticLabel.setText("Acoustic Phase")
        acousticLabel.setAlignment(QtCore.Qt.AlignCenter)
        camera0Label = QLabel(self)
        camera0Label.setText("Camera 1 View:")
        camera1Label = QLabel(self)
        camera1Label.setText("Camera 2 View:")

        self.tracking_button = QPushButton("Particle Tracking")
        # magnificationInput = QLineEdit(self)
        magnificationLabel = QLabel(self)
        magnificationLabel.setText("Magnification:")

        # Add buttons to a gridlayout within the 2nd column of the main grid
        # Layout is nx3 
        button_grid = QGridLayout()
        
        button_grid.addWidget(self.x1_slider,0,0)
        button_grid.addWidget(x1_label,0,1)
        button_grid.addWidget(self.direction_buttons[0],0,2)
        button_grid.addWidget(self.m1_slider,0,3)
        button_grid.addWidget(m1_label,0,4)
        button_grid.addWidget(self.direction_buttons[6],0,5)


        button_grid.addWidget(self.x2_slider,1,0)
        button_grid.addWidget(x2_label,1,1)
        button_grid.addWidget(self.direction_buttons[1],1,2)
        button_grid.addWidget(self.m2_slider,1,3)
        button_grid.addWidget(m2_label,1,4)
        button_grid.addWidget(self.direction_buttons[7],1,5)
        
        button_grid.addWidget(self.y1_slider,2,0)
        button_grid.addWidget(y1_label,2,1)
        button_grid.addWidget(self.direction_buttons[2],2,2)
        button_grid.addWidget(self.m3_slider,2,3)
        button_grid.addWidget(m3_label,2,4)
        button_grid.addWidget(self.direction_buttons[8],2,5)


        button_grid.addWidget(self.y2_slider,3,0)
        button_grid.addWidget(y2_label,3,1)
        button_grid.addWidget(self.direction_buttons[3],3,2)
        button_grid.addWidget(self.m4_slider,3,3)
        button_grid.addWidget(m4_label,3,4)
        button_grid.addWidget(self.direction_buttons[9],3,5)

        button_grid.addWidget(self.up,8,4)
        button_grid.addWidget(self.left,9,3)
        button_grid.addWidget(self.right,9,5)
        button_grid.addWidget(self.down,10,4)


        button_grid.addWidget(self.z1_slider,4,0)
        button_grid.addWidget(z1_label,4,1)
        button_grid.addWidget(self.direction_buttons[4],4,2)
        button_grid.addWidget(self.tracking_button, 4, 3)



        button_grid.addWidget(self.z2_slider,5,0)
        button_grid.addWidget(z2_label,5,1)
        button_grid.addWidget(self.direction_buttons[5],5,2)

        
        button_grid.addWidget(self.acousticButton,6,0)
        button_grid.addWidget(self.acousticReset, 6, 2)

        button_grid.addWidget(self.acousticPosButton, 7, 0)
        button_grid.addWidget(acousticLabel, 7, 1)
        button_grid.addWidget(self.acousticNegButton, 7, 2)

        button_grid.addWidget(magnificationLabel, 8, 0)
        button_grid.addWidget(self.calibrationButton, 8, 1)
        button_grid.addWidget(self.magneticButton, 8, 2)

        button_grid.addWidget(camera0Label, 9, 0)
        button_grid.addWidget(camera1Label, 9, 1)
        button_grid.addWidget(debug_button, 9, 2)

        button_grid.addWidget(self.camera0ComboBox, 10, 0)
        button_grid.addWidget(self.camera1ComboBox, 10, 1)
        button_grid.addWidget(self.com_port_box, 10, 2)

        button_grid.addWidget(self.debug_terminal, 11, 0, 3, 3,)

        self.buttonList = [self.up, self.down, self.left, self.right, self.magneticButton, 
                      self.calibrationButton, self.acousticPosButton, self.acousticNegButton, 
                      self.acousticButton, self.acousticReset, self.tracking_button]
        
        for button in self.direction_buttons:
            self.buttonList.append(button)

        for button in self.buttonList:
            # self.tracking_button.setEnabled(False)
            button.setEnabled(False)
        
        # Create a table widget
        self.table = QtWidgets.QTableWidget()
        self.table.setRowCount(10)
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Index","X Pos", "Y Pos", "Error"])

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
        
        sys.stdout = DebugStream(self.debug_terminal)

    
        print('Verifying camera work correctly')

    def joystick_button_press(self, button):
            maxwell_value = 100
            helmholtz_value = 100
            sleep_value = 0.2
            if button == self.up:
                self.coil3.set_direction(0)
                self.coil4.set_direction(0)
                self.coil3.set_pwm(helmholtz_value)
                self.coil4.set_pwm(helmholtz_value)
                self.y1_slider.setValue(helmholtz_value)
                self.y2_slider.setValue(helmholtz_value)
                time.sleep(sleep_value)
                self.coil9.set_direction(0)
                self.coil10.set_direction(1)
                self.coil9.set_pwm(maxwell_value)
                self.coil10.set_pwm(maxwell_value)
                self.m3_slider.setValue(maxwell_value)
                self.m4_slider.setValue(maxwell_value)
                self.direction_buttons[8].setText("+")
                self.direction_buttons[9].setText("-")
            elif button == self.down:
                self.coil3.set_direction(0)
                self.coil4.set_direction(0)
                self.coil3.set_pwm(helmholtz_value)
                self.coil4.set_pwm(helmholtz_value)
                self.y1_slider.setValue(helmholtz_value)
                self.y2_slider.setValue(helmholtz_value)
                time.sleep(sleep_value)
                self.coil9.set_direction(1)
                self.coil10.set_direction(0)
                self.coil9.set_pwm(maxwell_value)
                self.coil10.set_pwm(maxwell_value)
                self.m3_slider.setValue(maxwell_value)
                self.m4_slider.setValue(maxwell_value)
                self.direction_buttons[8].setText("-")
                self.direction_buttons[9].setText("+")
            elif button == self.right:
                self.coil1.set_direction(0)
                self.coil2.set_direction(0)
                self.coil1.set_pwm(helmholtz_value)
                self.coil2.set_pwm(helmholtz_value)
                self.x1_slider.setValue(helmholtz_value)
                self.x2_slider.setValue(helmholtz_value)
                time.sleep(sleep_value)
                self.coil7.set_direction(0)
                self.coil8.set_direction(1)
                self.coil7.set_pwm(maxwell_value)
                self.coil8.set_pwm(maxwell_value)
                self.m1_slider.setValue(maxwell_value)
                self.m2_slider.setValue(maxwell_value)
                self.direction_buttons[6].setText("+")
                self.direction_buttons[7].setText("-")
            else:
                self.coil1.set_direction(0)
                self.coil2.set_direction(0)
                self.coil1.set_pwm(helmholtz_value)
                self.coil2.set_pwm(helmholtz_value)
                self.x1_slider.setValue(helmholtz_value)
                self.x2_slider.setValue(helmholtz_value)
                time.sleep(sleep_value)
                self.coil7.set_direction(1)
                self.coil8.set_direction(0)
                self.coil7.set_pwm(maxwell_value)
                self.coil8.set_pwm(maxwell_value)
                self.m1_slider.setValue(maxwell_value)
                self.m2_slider.setValue(maxwell_value)
                self.direction_buttons[6].setText("-")
                self.direction_buttons[7].setText("+")
            print(f"button: {button.text()} pressed")

    def toggle_terminal(self):
        self.debug_terminal.setVisible(not self.debug_terminal.isVisible())

    def direction_switch(self, coil, input, button, slider):
        if button.text() == "-":
            button.setText("+")
        else:
            button.setText("-")
        self.change_slider(coil, input, button, slider)

    def connect_to_arduino(self, index):
        # Connect to selected serial port
        # Disconnect any previously opened serial port
        if self.arduino is not None and self.arduino.is_open:
            self.arduino.close()
        com_description = self.com_port_box.currentText()
        for port in serial.tools.list_ports.comports():
            if port.description == com_description:
                com_port = port.device
                break
        try:
            self.arduino = serial.Serial(com_port, 9600)
            self.setup_coils(self.arduino)
            self.setup_joystick()
            self.setup_inputs_and_sliders_and_buttons()
            print("Arduino Connected")
            for button in self.buttonList:
                button.setEnabled(True)
        except serial.SerialException as e:
            print(f"COM Port selected is not an arduino: {e}")


    # All buttons that require communication to the arduino, need to go here. 
    # Or else the GUI will crash, due to an arduino not be connected to communicate with the coils
    def setup_inputs_and_sliders_and_buttons(self):
        coils = [self.coil1, self.coil2, self.coil3, self.coil4, self.coil5, self.coil6, self.coil7, self.coil8, self.coil9, self.coil10]
        sliders = [self.x1_slider, self.x2_slider, self.y1_slider, self.y2_slider, self.z1_slider, self.z2_slider, self.m1_slider, self.m2_slider, self.m3_slider, self.m4_slider]
        inputs = [self.x1_input, self.x2_input, self.y1_input, self.y2_input, self.z1_input, self.z2_input, self.m1_input, self.m2_input, self.m3_input, self.m4_input]

        for coil, slider, input, dir_butt in zip(coils, sliders, inputs, self.direction_buttons):
            dir_butt.clicked.connect(partial(self.direction_switch, coil, input, dir_butt, slider))
            slider.sliderReleased.connect(partial(self.change_slider, coil, input, dir_butt, slider))
            slider.setMaximum(255)
            slider.setMinimum(0)

            input.setFixedWidth(50)
            # input.returnPressed.connect(partial(self.set_slider, slider, input, coil))
            # input.editingFinished.connect(partial(self.set_slider, slider, input, coil))

        self.magneticButton.clicked.connect(self.coilsOffClick)
        self.acousticReset.clicked.connect(partial(self.acousticButtonPress, "R\n"))
        self.acousticPosButton.clicked.connect(partial(self.acousticButtonPress, "U\n"))
        self.acousticButton.clicked.connect(partial(self.onAcousticClick, self.acousticButton))
        self.acousticButton.clicked.connect(partial(self.acousticButtonPress, "E\n"))
        self.acousticNegButton.clicked.connect(partial(self.acousticButtonPress, "D\n"))

    
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
        self.tracking_button.clicked.connect(partial(self.enable_tracking,self.zero))

    def camera1Change(self, index):
        # print("Text changed:", s)
        print("View 2 was changed to " + str(index - 1))
        if self.camera1 is not None:
            self.my_grid.removeWidget(self.one.get_video_frame())
        self.camera1 = self.availableCameras[index - 1]
        self.one = CameraWidget(self.screen_width//3, self.screen_height//3, self.table, self.camera1)
        self.my_grid.addWidget(self.one.get_video_frame(),1,0,1,1)
        # self.tracking_button.clicked.connect(partial(self.enable_tracking,self.one))
    

    def calibration_button(self, camera):
        self.distance = 0
        if camera.magButton == True:
            camera.distance = np.linalg.norm(np.array([camera.start_x, camera.start_y])-np.array([camera.end_x, camera.end_y]))
            print(f"Distance: {camera.distance}, Magbutton: {camera.magButton}")
            camera.magButton = False
            camera.clicked = 0
            camera.start_x = 0
            camera.start_y = 0
            self.calibrationButton.setText("Start Calibration")
        else:
            camera.magButton = True
            self.calibrationButton.setText("End Calibration")

    def enable_tracking(self, camera):
        if camera.tracking_enabled:
            camera.tracking_enabled = False
            camera.particles = {}
        else:
            camera.tracking_enabled = True
        print(f"Tracking: {camera.tracking_enabled}")

    def change_slider(self, coil, input, dir_butt, slider):
        value = slider.value()
        if (dir_butt.text() == "+"):
            coil.set_direction(0)
        else:
            coil.set_direction(1)
        print(dir)
        print("value changed")

        coil.set_pwm(abs(value))
        input.setText(str(value))

    # def set_slider(self, slider, input, coil):
    #     text = input.text()
    #     if text == "":
    #         value = 0
    #     elif text.startswith("-"):
    #         coil.set_direction(0)
    #         value_str = text[1:]
    #         if len(value_str) > 0:
    #             value = -int(value_str)
    #         else:
    #             value = 0
    #     else:
    #         coil.set_direction(1)
    #         value = int(text)
    #     slider.setValue(value)
    #     coil.set_pwm(abs(value))

    def acousticButtonPress(self, command):
        try:
            self.arduino.write(command.encode())
        except serial.SerialException as e:
            print(f"Failed to write acoustics to arduino: {e}")

        print("clicked")
        # time.sleep(2)
        # text = self.arduino.readline()
        # print(text)
    
    def setup_joystick(self):
        self.up.pressed.connect(partial(self.joystick_button_press, self.up))
        self.down.pressed.connect(partial(self.joystick_button_press, self.down))
        self.left.pressed.connect(partial(self.joystick_button_press, self.left))
        self.right.pressed.connect(partial(self.joystick_button_press, self.right))
        

    def setup_coils(self, arduino):
        # TODO: ADD M, X and R values for maxwell coils
        m = [0.009874387,
            0.010136336,
            0.013625919,
            0.013743873,
            0.016740196,
            0.016591605,
            0,
            0,
            0,
            0]
        x = [-0.061568627,
            -0.063333333,
            -0.094705882,
            -0.082156863,
            -0.090980392,
            -0.0901960780,
            0,
            0,
            0,
            0]
        r = [9.3, 8.9, 6.7, 6.8, 5.5, 5.5, 2.5, 2.5, 2.5, 2.5]

        self.coil1 = Coil("X", 1, r[0], m[0], x[0], arduino)
        self.coil2 = Coil("X", 2, r[1], m[1], x[1], arduino)
        self.coil3 = Coil("Y", 1, r[2], m[2], x[2], arduino)
        self.coil4 = Coil("Y", 2, r[3], m[3], x[3], arduino)
        
        # Disabled due to motor controller failure
        self.coil5 = Coil("Z", 1, r[4], m[4], x[4], arduino)
        self.coil6 = Coil("Z", 2, r[5], m[5], x[5], arduino)

        self.coil7 = Coil("M", 1, r[6], m[6], x[6], arduino)
        self.coil8 = Coil("M", 2, r[7], m[7], x[7], arduino)
        self.coil9 = Coil("M", 3, r[8], m[8], x[8], arduino)
        self.coil10 = Coil("M", 4, r[9], m[9], x[9], arduino)

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
        coils = [self.coil1, self.coil2, self.coil3, self.coil4, self.coil5, self.coil6, self.coil7, self.coil8, self.coil9, self.coil10]
        sliders = [self.x1_slider, self.x2_slider, self.y1_slider, self.y2_slider, self.z1_slider, self.z2_slider, self.m1_slider, self.m2_slider, self.m3_slider, self.m4_slider]
        inputs = [self.x1_input, self.x2_input, self.y1_input, self.y2_input, self.z1_input, self.z2_input, self.m1_input, self.m2_input, self.m3_input, self.m4_input]
        for i in range(len(coils)):
            coils[i].set_pwm(0)
            sliders[i].setValue(0)
            inputs[i].setText("0")
        print("Magnetics Off") # Debugging code
    

class DebugStream:
    """
    A class that redirects the standard output to a QTextStream
    """
    def __init__(self, text_widget):
        self.text_widget = text_widget
        self.cursor = self.text_widget.textCursor()

    def write(self, text):
        self.cursor.movePosition(self.cursor.End)
        self.cursor.insertText(text)
        self.text_widget.ensureCursorVisible()
    def flush(self):
        return None

if __name__ == '__main__':
    

    # Create main application window
    app = QApplication([])
    main_app = App()
    main_app.show()

    QtWidgets.QShortcut(QtGui.QKeySequence('Ctrl+Q'), main_app, exit_application)

    if(sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()