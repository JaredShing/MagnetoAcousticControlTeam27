from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QMainWindow, QApplication, QLabel, QVBoxLayout, QPushButton, QGridLayout
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
from enum import Enum
from PyQt5.QtWidgets import QLineEdit
from PyQt5 import QtCore

# This code starts 2 cameras on separate threads to prevent frame lag
class CameraWidget(QtWidgets.QWidget):
    """Independent camera feed
    Uses threading to grab IP camera frames in the background
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

        print('Started camera: {}'.format(self.camera_stream_link))

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

            # Add timestamp to cameras
            # cv2.rectangle(self.frame, (self.screen_width-190,0), (self.screen_width,50), color=(0,0,0), thickness=-1)
            # cv2.putText(self.frame, datetime.now().strftime('%H:%M:%S'), (self.screen_width-185,37), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255,255,255), lineType=cv2.LINE_AA)
            
            self.add_contours()
            
            # Convert to pixmap and set to video frame
            self.img = QtGui.QImage(self.frame, self.frame.shape[1], self.frame.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
            self.pix = QtGui.QPixmap.fromImage(self.img)
            self.video_frame.setPixmap(self.pix)

    def get_video_frame(self):
        return self.video_frame

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

class direction(Enum):
    FORWARD = 1
    BACKWARD = -1

class Coil():
    def __init__(self, axis, num, resistance, arduino):
        self.axis = axis
        self.num = num
        self.resistance = resistance
        self.voltage = 24
        self.pwm_value = 0
        self.PWM_MAX = 255
        self.direction = direction.FORWARD
        self.arduino = arduino
    
    # Increase the pwm value being sent to the coil
    def increment_pwm_value(self):
        self.pwm_value += 1

        string_to_send = f"{self.axis}{self.num}{str(self.pwm_value).zfill(3)}"
        self.arduino.write(string_to_send.encode())
        print(string_to_send)

    # Decreae the pwm value being sent to the coil
    def decrease_pwm_value(self):
        self.pwm_value -= 1

        string_to_send = f"{self.axis}{self.num}{str(self.pwm_value).zfill(3)}"
        self.arduino.write(string_to_send.encode())
        print(string_to_send)
    
    # return the calculated current value based on the pwm value
    def get_current_value(self):
        return self.pwm_value/self.PWM_MAX*self.voltage/self.resistance

    # T0 change the direction we send a request to the arduino via serial.
    # The serial request sends the coil axis {x, y, z}, then the direction {f, b}
    # Foe example to change coil 1 forward, the request is "Xf"
    def set_direction(self, direction_request):
        self.direction = direction_request
        if self.direction == direction.BACKWARD:
            self.arduino.write(f"{self.axis}f".encode())
        else:
            self.arduino.write(f"{self.axis}b".encode())


class App(QMainWindow):
    def __init__(self):
        super().__init__()

        self.xPos = 0
        self.yPos = 0
        self.zPos = 0
        self.acoustic = 0
        self.mag = 0

        print('Creating Arduino Connection')
        self.arduino = serial.Serial('COM7',9600) #Create Serial port object called arduinoSerialData
        self.setup_coils(self.arduino)

        self.setWindowTitle("Magneto-Acoustic Control GUI")

        # Create buttons to control magnetics and acoustics
        x1PosButton = QPushButton('+', self)
        x1PosButton.setToolTip('Increase the x1 field')
        x1PosButton.clicked.connect(partial(self.onCoilClick, self.coil1))

        x1NegButton = QPushButton('-', self)
        x1NegButton.setToolTip('Decrease the x1 field')
        x1NegButton.clicked.connect(partial(self.onCoilClick, self.coil2))

        y1PosButton = QPushButton('+', self)
        y1PosButton.setToolTip('Increase the y1 field')
        y1PosButton.clicked.connect(partial(self.onCoilClick, self.coil3))

        y1NegButton = QPushButton('-', self)
        y1NegButton.setToolTip('Decrease the y1 field')
        y1NegButton.clicked.connect(partial(self.onCoilClick, self.coil4))

        z1PosButton = QPushButton('+', self)
        z1PosButton.setToolTip('Increase the z1 field')
        z1PosButton.clicked.connect(partial(self.onCoilClick, self.coil5))

        z1NegButton = QPushButton('-', self)
        z1NegButton.setToolTip('Decrease the z1 field')
        z1NegButton.clicked.connect(partial(self.onCoilClick, self.coil6))

        x2PosButton = QPushButton('+', self)
        x2PosButton.setToolTip('Increase the x2 field')
        x2PosButton.clicked.connect(partial(self.onCoilClick, self.coil1))

        x2NegButton = QPushButton('-', self)
        x2NegButton.setToolTip('Decrease the x2 field')
        x2NegButton.clicked.connect(partial(self.onCoilClick, self.coil2))

        y2PosButton = QPushButton('+', self)
        y2PosButton.setToolTip('Increase the y2 field')
        y2PosButton.clicked.connect(partial(self.onCoilClick, self.coil3))

        y2NegButton = QPushButton('-', self)
        y2NegButton.setToolTip('Decrease the y2 field')
        y2NegButton.clicked.connect(partial(self.onCoilClick, self.coil4))

        z2PosButton = QPushButton('+', self)
        z2PosButton.setToolTip('Increase the z2 field')
        z2PosButton.clicked.connect(partial(self.onCoilClick, self.coil5))

        z2NegButton = QPushButton('-', self)
        z2NegButton.setToolTip('Decrease the z2 field')
        z2NegButton.clicked.connect(partial(self.onCoilClick, self.coil6))

        acousticPosButton = QPushButton('+', self)
        acousticPosButton.setToolTip('Increase Acoustic Field Intensity')
        acousticPosButton.clicked.connect(partial(self.onAcousticClick, 1))

        acousticNegButton = QPushButton('-', self)
        acousticNegButton.setToolTip('Decrease Acoustic Field Intensity')
        acousticNegButton.clicked.connect(partial(self.onAcousticClick, -1))

        acousticButton = QPushButton('Acoustics On/Off', self)
        acousticButton.setToolTip('Turn on/off acoustics')
        acousticButton.clicked.connect(partial(self.onMagClick))

        acousticReset = QPushButton('Acoustic Reset', self)
        acousticReset.setToolTip('Reset Acoustic Intensity to 0')
        acousticReset.clicked.connect(partial(self.onResetClick))

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
        acousticLabel.setText("Acoustic Intensity")
        acousticLabel.setAlignment(QtCore.Qt.AlignCenter)

         # Creating an input box with label
        magnificationInput = QLineEdit(self)
        magnificationLabel = QLabel(self)
        magnificationLabel.setText("Magnification:")

        # Add buttons to a gridlayout within the 2nd column of the main grid
        button_grid = QGridLayout()
        button_grid.addWidget(x1PosButton,0,0)
        button_grid.addWidget(x1Label,0,1)
        button_grid.addWidget(x1NegButton,0,2)
        button_grid.addWidget(y1PosButton,1,0)
        button_grid.addWidget(y1Label,1,1)
        button_grid.addWidget(y1NegButton,1,2)
        button_grid.addWidget(z1PosButton,2,0)
        button_grid.addWidget(z1Label,2,1)
        button_grid.addWidget(z1NegButton,2,2)
        button_grid.addWidget(x2PosButton,3,0)
        button_grid.addWidget(x2Label,3,1)
        button_grid.addWidget(x2NegButton,3,2)
        button_grid.addWidget(y2PosButton,4,0)
        button_grid.addWidget(y2Label,4,1)
        button_grid.addWidget(y2NegButton,4,2)
        button_grid.addWidget(z2PosButton,5,0)
        button_grid.addWidget(z2Label,5,1)
        button_grid.addWidget(z2NegButton,5,2)
        button_grid.addWidget(acousticButton,6,0)
        button_grid.addWidget(acousticReset, 6, 2)
        button_grid.addWidget(acousticPosButton, 7, 0)
        button_grid.addWidget(acousticLabel, 7, 1)
        button_grid.addWidget(acousticNegButton, 7, 2)
        # button_grid.addWidget(magnificationLabel, 5, 0)
        # button_grid.addWidget(magnificationInput, 5, 1, 1, 2)
        
        
        # Create a table widget
        table = QtWidgets.QTableWidget()
        table.setRowCount(10)
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(["Index","X Pos", "Y Pos"])

        cw = QtWidgets.QWidget()
        my_grid = QtWidgets.QGridLayout()
        cw.setLayout(my_grid)
        self.setCentralWidget(cw)
        
        # Dynamically determine screen width/height
        screen_width = QtWidgets.QApplication.desktop().screenGeometry().width()
        screen_height = QtWidgets.QApplication.desktop().screenGeometry().height()
        
        # Stream links
        camera0 = 1
        camera1 = 0
        
        # Create camera widgets
        print('Creating Camera Widgets...')
        zero = CameraWidget(screen_width//3, screen_height//3, table, camera0)
        one = CameraWidget(screen_width//3, screen_height//3, table, camera1)
        while zero.online is False:
              time.sleep(1)
        # Add widgets to layout
        print('Adding widgets to layout...')
        my_grid.addWidget(zero.get_video_frame(),0,0,1,2)
        my_grid.addWidget(one.get_video_frame(),1,0,1,1)
        my_grid.addWidget(table,0,2,1,3)
        my_grid.addLayout(button_grid,0,5,1,2)
        print(my_grid.columnCount())
        
        print('Verifying camera work correctly')


    # when the direction buttons are clicked
    def onCoilClick(self, coil):
        coil.increment_pwm_value()

    def onAcousticClick(self, value):
        self.acoustic = self.acoustic + value
        print('Acoustic intensity changed value ' + str(self.acoustic))

    def onResetClick(self):
        self.acoustic = 0
        print("Acoustic = " + str(self.acoustic))
    
    def setup_coils(self, arduino):
        self.coil1 = Coil("X", 1, 5.5, arduino)
        self.coil2 = Coil("X", 2, 5.6, arduino)
        self.coil3 = Coil("Y", 1, 6.2, arduino)
        self.coil4 = Coil("Y", 2, 6.3, arduino)
        self.coil5 = Coil("Z", 1, 7.7, arduino)
        self.coil6 = Coil("Z", 2, 7.5, arduino)


    def onMagClick(self):
        if self.mag == 0:
            self.mag = 1
        else:
            self.mag = 0
        print("Mag = " + str(self.mag))

if __name__ == '__main__':
    
    # Create main application window
    app = QApplication([])
    main_app = App()
    main_app.show()

    QtWidgets.QShortcut(QtGui.QKeySequence('Ctrl+Q'), main_app, exit_application)

    if(sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()