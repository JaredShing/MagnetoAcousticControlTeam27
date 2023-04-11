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

        print('Started camera: {}'.format(self.camera_stream_link))

        # self.painter = QPainter(self)

    def load_network_stream(self):
        """Verifies stream link and open new stream if valid"""

        def load_network_stream_thread():
            if self.verify_network_stream(self.camera_stream_link):
                self.capture = cv2.VideoCapture(self.camera_stream_link)
                self.online = True
        self.load_stream_thread = Thread(target=load_network_stream_thread, args=())
        self.load_stream_thread.daemon = True
        self.load_stream_thread.start()

    def verify_network_stream(self, link):
        """Attempts to receive a frame from given link"""
        cap = cv2.VideoCapture(link)
        if not cap.isOpened():
            return False
        cap.release()
        return True

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
                else:
                    # Attempt to reconnect
                    print('attempting to reconnect', self.camera_stream_link)
                    self.load_network_stream()
                    self.spin(2)
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

            # Keep frame aspect ratio
            if self.maintain_aspect_ratio:
                self.frame = imutils.resize(frame, width=self.screen_width)
            # Force resize
            else:
                self.frame = cv2.resize(frame, (self.screen_width, self.screen_height))
            # Add green line to indicte clicked point
            if self.magButton:
                if self.clicked == 1:
                    cv2.line(self.frame, (self.start_x, self.start_y), (self.x, self.y), color=(0, 255, 0), thickness=2)
                if self.clicked == 2:
                    cv2.line(self.frame, (self.start_x, self.start_y), (self.end_x, self.end_y), color=(0, 255, 0), thickness=2)

                    

            self.add_contours()
            
            # Convert to pixmap and set to video frame
            self.img = QtGui.QImage(self.frame, self.frame.shape[1], self.frame.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
            self.pix = QtGui.QPixmap.fromImage(self.img)
            self.video_frame.setPixmap(self.pix)

    def get_video_frame(self):
        return self.video_frame
    
    def mouse_callback(self, event):
        if self.magButton:
            self.clicked += 1
            if self.clicked == 2:
                self.end_x = event.pos().x()
                self.end_y = event.pos().y()
                self.video_frame.setMouseTracking(False)
            else:
                self.video_frame.setMouseTracking(True)
                self.start_x = event.pos().x()
                self.start_y = event.pos().y()
        print("Mouse clicked at x={}, y={}, clicked = {}".format(self.start_x, self.start_y, self.clicked))
        


    def mouse_move_callback(self, event):
        self.x = event.pos().x()
        self.y = event.pos().y()

    def add_contours(self):
        # Create a color range
        gray = np.array([180, 255, 50])
        black = np.array([0, 0, 0])

        """Convert from an opencv image to QPixmap"""
        # rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

        mask = cv2.inRange(hsv, black, gray)

        numObjects = 0

        dilated = self.dilatation(self.frame)
        hsvTest = cv2.cvtColor(dilated, cv2.COLOR_BGR2HSV)
        maskTest = cv2.inRange(hsvTest, black, gray)
        contours, hierarchy = cv2.findContours(maskTest, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.positions.clear()
        for object in contours:
            objectArea = cv2.contourArea(object)
            if objectArea > 400:
                x, y, width, height = cv2.boundingRect(object)
                cv2.rectangle(self.frame, (x, y), (x + width, y + height), (255, 0, 0), 2)
                cv2.drawContours(self.frame, object, -1, (0, 0, 255), 2)
                self.positions[numObjects] = (x,y)
                numObjects += 1
                font = cv2.FONT_HERSHEY_PLAIN
                midX = x + width / 2
                midY = y + height / 2
                cv2.putText(self.frame, str(numObjects), (int(midX), int(midY)), font, 3, (255, 0, 0), 2, cv2.LINE_AA)
        self.update_table(self.data_table, self.positions)
                
    def update_table(self, tableWidget, data):
    # Clear the table
        tableWidget.clearContents()

        # Add the updated dictionary data to the table
        for i, (index, (x, y)) in enumerate(data.items()):
            tableWidget.setItem(i, 0, QtWidgets.QTableWidgetItem(str(index)))
            tableWidget.setItem(i, 1, QtWidgets.QTableWidgetItem(str(x)))
            tableWidget.setItem(i, 2, QtWidgets.QTableWidgetItem(str(y)))
                    
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
        self.arduino.write(string_to_send.encode())
        print(string_to_send)

    # return the calculated current value based on the pwm value
    def get_current_value(self):
        return self.current

    # T0 change the direction we send a request to the arduino via serial.
    # The serial request sends the coil axis {x, y, z}, then the direction {f, b}
    # Foe example to change coil 1 forward, the request is "Xf"
    def set_direction(self, direction_request):
        self.direction = direction_request

    def calc_current(self):
        self.current = self.pwm_value*self.m + self.b


class App(QMainWindow):
    def __init__(self):
        super().__init__()

        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.acoustic = 0
        self.acousticOnOff = 0
        self.magneticsOnOff = 0

        print('Creating Arduino Connection')
        self.arduino = None #Create Serial port object called arduinoSerialData

        self.setWindowTitle("Magneto-Acoustic Control GUI")

        self.camera0ComboBox = QComboBox()
        self.availableCameras = self.returnCameraIndexes()
        print("Avaiable Camreas")
        print(self.availableCameras)
        self.camera0ComboBox.addItem("Select Camera")
        for i in self.availableCameras:
            self.camera0ComboBox.addItem("Camera " + str(i))
        self.camera0ComboBox.currentIndexChanged.connect(self.camera0Change)

        self.camera1ComboBox = QComboBox()
        self.camera1ComboBox.addItem("Select Camera")
        for i in self.availableCameras:
            self.camera1ComboBox.addItem("Camera " + str(i))
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
        magneticButton.clicked.connect(self.onMagneticClick)

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

         # Creating an input box with label
        # magnificationInput = QLineEdit(self)
        magnificationLabel = QLabel(self)
        magnificationLabel.setText("Magnification:")

        # Add buttons to a gridlayout within the 2nd column of the main grid
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
        print('Creating Camera Widgets...')
        self.zero = None
        self.one = None

        # Create magnification button

        self.magnificationButton = QPushButton('Start Calibration', self)
        self.magnificationButton.setToolTip('Calibration Button')
        button_grid.addWidget(self.magnificationButton, 8, 1)

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
            self.my_grid.removeWidget(self.zero.get_video_frame())
        self.camera0 = self.availableCameras[index - 1]
        self.zero = CameraWidget(self.screen_width//3, self.screen_height//3, self.table, self.camera0)
        self.my_grid.addWidget(self.zero.get_video_frame(),0,0,1,2)
        self.magnificationButton.clicked.connect(partial(self.calibration_button, self.zero))

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
            self.distance = ((camera.start_x-camera.end_x)**2-(camera.start_y-camera.end_y)**2)**(1/2)
            print(f"Distance: {self.distance}, Magbutton: {camera.magButton}")
            camera.magButton = False
            camera.clicked = 0
            camera.start_x = 0
            camera.start_y = 0
            self.magnificationButton.setText("Start Calibration")
        else:
            camera.magButton = True
            self.magnificationButton.setText("End Calibration")


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

        self.coil1 = Coil("X", 1, r[0], m[0], x[0], arduino)
        self.coil2 = Coil("X", 2, r[1], m[1], x[1], arduino)
        self.coil3 = Coil("Y", 1, r[2], m[2], x[2], arduino)
        self.coil4 = Coil("Y", 2, r[3], m[3], x[3], arduino)
        self.coil5 = Coil("Z", 1, r[4], m[4], x[4], arduino)
        self.coil6 = Coil("Z", 2, r[5], m[5], x[5], arduino)

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

    def onMagneticClick(self):
        coils = [self.coil1, self.coil2, self.coil3, self.coil4, self.coil5, self.coil6]
        sliders = [self.x1_slider, self.x2_slider, self.y1_slider, self.y2_slider, self.z1_slider, self.z2_slider]
        inputs = [self.x1_input, self.x2_input, self.y1_input, self.y2_input, self.z1_input, self.z2_input]
        for i in range(len(coils)):
            coils[i].set_pwm(0)
            sliders[i].setValue(0)
            inputs[i].setText("0")
        print("Magnetics Off") # Debugging code

if __name__ == '__main__':
    
    # Create main application window
    app = QApplication([])
    main_app = App()
    main_app.show()

    QtWidgets.QShortcut(QtGui.QKeySequence('Ctrl+Q'), main_app, exit_application)

    if(sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()