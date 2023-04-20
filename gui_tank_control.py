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
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import cProfile
import pstats


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
            self.add_contours()

            if self.magButton:
                if self.clicked == 1:
                    if self.x + self.y == 0:
                        self.x = self.start_x
                        self.y = self.start_y
                    cv2.line(self.frame, (self.start_x, self.start_y), (self.x, self.y), color=(0, 255, 0), thickness=2)
                if self.clicked == 2:
                    cv2.line(self.frame, (self.start_x, self.start_y), (self.end_x, self.end_y), color=(0, 255, 0), thickness=2)
            
            #changing
            # Was getting a complex number error if calibration button was hit before ending 2nd point
            if not isinstance(self.distance, complex):
                self.scale_text =  f"Scale: {round(self.distance/10,3)}px/mm"

            cv2.putText(frame, self.scale_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

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
        gray = np_array([180, 255, 50])
        black = np_array([0, 0, 0])

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
            print(f"COM Port selected is not an arduino: ")
        # print(string_to_send)

    # return the calculated current value based on the pwm value
    def get_current_value(self):
        return self.current

    # Set the direction of the coil
    def set_direction(self, direction_request):
        self.direction = direction_request

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

        com_ports = serial.tools.list_ports.comports()
        com_ports_names = [port.device for port in com_ports]
        self.com_port_box = QComboBox()
        self.com_port_box.addItems(com_ports_names)
        self.com_port_box.currentIndexChanged.connect(self.connect_to_arduino)

        self.serial_data = ""


        self.plot_button = QPushButton("Plot")
        self.plot_button.clicked.connect(self.show_plot_window)

        self.toolbar = self.addToolBar("Main Toolbar")
        self.toolbar.addWidget(self.plot_button)

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

        self.up = QPushButton("Forward")
        self.down = QPushButton("Back")
        self.left = QPushButton("Rotate -")
        self.right = QPushButton("Rotate +")


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


        button_grid.addWidget(self.z2_slider,5,0)
        button_grid.addWidget(z2_label,5,1)
        button_grid.addWidget(self.direction_buttons[5],5,2)

        
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
        button_grid.addWidget(debug_button, 9, 2)

        button_grid.addWidget(self.camera0ComboBox, 10, 0)
        button_grid.addWidget(self.camera1ComboBox, 10, 1)
        button_grid.addWidget(self.com_port_box, 10, 2)

        button_grid.addWidget(self.debug_terminal, 11, 0, 3, 3,)
        
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
        
        sys.stdout = DebugStream(self.debug_terminal)

    
        print('Verifying camera work correctly')

    def update(self):
        try:
            if self.arduino.in_waiting > 0:
                if len(self.serial_data) > 10000: # if buffer is too full, clear it
                    self.arduino.flushInput()
                    self.serial_data = ""
                    print("Buffer cleared")
                self.serial_data += self.arduino.read(self.arduino.in_waiting).decode()
                if '\n' in self.serial_data:
                    lines = self.serial_data.split('\n')
                    self.serial_data = lines[-1]
                    for line in lines[:-1]:
                        print(line)
        except UnicodeDecodeError:
            print("UnicodeDecodeError")
            self.arduino.flushInput()
            self.serial_data = ""

    def show_plot_window(self):
        # Create and show an instance of the PlotWindow class
        self.plot_window = PlotWindow(self.button_values)
        self.plot_window.show()

    def joystick_button_press(self, button):
        self.button_values[button]["timer"] = QtCore.QTimer()
        self.button_values[button]["timer"].timeout.connect(partial(self.increment_value, button))
        self.button_values[button]["timer"].start(300)  # 100 ms interval

    def joystick_button_release(self, button):
        self.button_values[button]["timer"].stop()
        self.button_values[button]["timer"].timeout.disconnect()
        self.button_values[button]["timer"] = QtCore.QTimer()
        self.button_values[button]["timer"].timeout.connect(partial(self.decay_value, button))
        self.button_values[button]["timer"].start(300)  # 100 ms interval

    def increment_value(self, button):
        self.button_values[button]["value"] = min(round(self.button_values[button]["value"])+20, 255)
        print(f'{button.text()}: {self.button_values[button]["value"]}')
        self.button_values[button]["coils"][0].set_pwm(self.button_values[button]["value"])
        time.sleep(0.01)
        self.button_values[button]["coils"][1].set_pwm(self.button_values[button]["value"])
        self.button_values[button]["value_history"].append(self.button_values[button]["value"])
        self.button_values[button]["time_history"].append(datetime.now())
        if len(self.button_values[button]["value_history"]) > 200:
            self.button_values[button]["value_history"].pop(0)
            self.button_values[button]["time_history"].pop(0)

    def decay_value(self, button):
        if self.button_values[button]["value"] <= 5:
            self.button_values[button]["timer"].stop()
            self.button_values[button]["timer"].timeout.disconnect()
        else:
            self.decay_constant = 1.5
            self.button_values[button]["value"] = self.button_values[button]["value"] * math.exp(-self.decay_constant * 100 / 1000)  # decay over 100 ms
            print(f'{button.text()}: {round(self.button_values[button]["value"])}')
            self.button_values[button]["coils"][0].set_pwm(round(self.button_values[button]["value"]))
            time.sleep(0.01)
            self.button_values[button]["coils"][1].set_pwm(round(self.button_values[button]["value"]))
            self.button_values[button]["value_history"].append(self.button_values[button]["value"])
            self.button_values[button]["time_history"].append(datetime.now())
            if len(self.button_values[button]["value_history"]) > 200:
                self.button_values[button]["value_history"].pop(0)
                self.button_values[button]["time_history"].pop(0)

    def toggle_terminal(self):
        self.debug_terminal.setVisible(not self.debug_terminal.isVisible())

    def direction_switch(self, coil, input, button, slider):
        if button.text() == "-":
            button.setText("+")
        else:
            button.setText("-")
        self.change_slider(coil, input, button, slider)

    def connect_to_arduino(self, index):
        # Disconnect any previously opened serial port
        if self.arduino is not None and self.arduino.is_open:
            self.arduino.close()
        
        # Connect to selected serial port
        com_port = self.com_port_box.currentText()
        try:
            self.arduino = serial.Serial(com_port, 9600, timeout=1)
            self.setup_coils(self.arduino)
            self.setup_joystick()
            self.setup_inputs_and_sliders()
            self.timer2 = QtCore.QTimer(self)
            self.timer2.timeout.connect(self.update)
            self.timer2.start(100)  # call update every 100 ms
            print("Arduino_Connected")
        except serial.SerialException as e:
            print(f"COM Port selected is not an arduino: {e}")



    def setup_inputs_and_sliders(self):
        coils = [self.coil1, self.coil2, self.coil3, self.coil4, self.coil5, self.coil6, self.coil7, self.coil8, self.coil9, self.coil10]
        sliders = [self.x1_slider, self.x2_slider, self.y1_slider, self.y2_slider, self.z1_slider, self.z2_slider, self.m1_slider, self.m2_slider, self.m3_slider, self.m4_slider]
        inputs = [self.x1_input, self.x2_input, self.y1_input, self.y2_input, self.z1_input, self.z2_input, self.m1_input, self.m2_input, self.m3_input, self.m4_input]

        for coil, slider, input, dir_butt in zip(coils, sliders, inputs, self.direction_buttons):
            dir_butt.clicked.connect(partial(self.direction_switch, coil, input, dir_butt, slider))
            slider.sliderReleased.connect(partial(self.change_slider, coil, input, dir_butt, slider))
            slider.setMaximum(255)
            slider.setMinimum(0)

            input.setFixedWidth(50)
            input.returnPressed.connect(partial(self.set_slider, slider, input, coil))
            input.editingFinished.connect(partial(self.set_slider, slider, input, coil))
    
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
        self.up.released.connect(partial(self.joystick_button_release, self.up))
        self.down.pressed.connect(partial(self.joystick_button_press, self.down))
        self.down.released.connect(partial(self.joystick_button_release, self.down))
        self.left.pressed.connect(partial(self.joystick_button_press, self.left))
        self.left.released.connect(partial(self.joystick_button_release, self.left))
        self.right.pressed.connect(partial(self.joystick_button_press, self.right))
        self.right.released.connect(partial(self.joystick_button_release, self.right))

        self.button_values = {
        self.up: {"value": 0, "timer": None, "coils": [self.coil1, self.coil2], "value_history": [], "time_history": []},
        self.down: {"value": 0, "timer": None, "coils": [self.coil3, self.coil4], "value_history": [], "time_history": []},
        self.left: {"value": 0, "timer": None, "coils": [self.coil5, self.coil6], "value_history": [], "time_history": []},
        self.right: {"value": 0, "timer": None, "coils": [self.coil7, self.coil8], "value_history": [], "time_history": []}
        }

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
    
class PlotWindow(QtWidgets.QWidget):
    def __init__(self, button_values):
        super().__init__()

        self.button_values = button_values

        self.setWindowTitle("Button Value Plot")
        self.setGeometry(200, 200, 800, 600)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)

        self.button_layout = QtWidgets.QHBoxLayout()
        
        self.plot_button = QPushButton("Plot All")
        self.plot_button.clicked.connect(self.plot_all_button_clicked)

        self.clear_button = QtWidgets.QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_plot)

        self.button_layout.addWidget(self.plot_button)
        self.button_layout.addWidget(self.clear_button)
        

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.canvas)
        self.layout.addLayout(self.button_layout)

        self.setLayout(self.layout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.plot_all_button_clicked)
        self.timer.start(200)


    def plot_all_button_clicked(self):
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Button Value")
        ax.set_title("Button Values over Time")

        for button in self.button_values:
            values = self.button_values[button]["value_history"]
            times = self.button_values[button]["time_history"]
            ax.plot(times, values, label=button.text())

        ax.legend()
        self.canvas.draw()
    def clear_plot(self):
        for button in self.button_values:
            self.button_values[button]["value_history"].clear()
            self.button_values[button]["time_history"].clear()
    
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
    
    # create a profile object
    profiler = cProfile.Profile()

    # start profiling
    profiler.enable()

    # Create main application window
    app = QApplication([])
    main_app = App()
    main_app.show()

    QtWidgets.QShortcut(QtGui.QKeySequence('Ctrl+Q'), main_app, exit_application)

    if(sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()

    # stop profiling
    profiler.disable()

    # save profiling results to file
    with open('profile_stats.txt', 'w') as f:
        stats = pstats.Stats(profiler, stream=f)
        stats.sort_stats('cumulative').print_stats()