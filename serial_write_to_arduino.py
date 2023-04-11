import serial
import time

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

def setup_coils(arduino):
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

    coil1 = Coil("X", 1, r[0], m[0], x[0], arduino)
    coil2 = Coil("X", 2, r[1], m[1], x[1], arduino)
    coil3 = Coil("Y", 1, r[2], m[2], x[2], arduino)
    coil4 = Coil("Y", 2, r[3], m[3], x[3], arduino)
    coil5 = Coil("Z", 1, r[4], m[4], x[4], arduino)
    coil6 = Coil("Z", 2, r[5], m[5], x[5], arduino)
    return [coil1, coil2, coil3, coil4, coil5, coil6]

arduino = serial.Serial('COM5',9600) #Create Serial port object called arduinoSerialData
print("Arduino connected")
coils = setup_coils(arduino)
while True:
    pwm = 255
    time_val = 1
    coils[0].set_pwm(pwm)
    coils[1].set_pwm(-pwm)
    coils[2].set_pwm(0)
    coils[3].set_pwm(0)
    time.sleep(time_val)
    coils[0].set_pwm(0)
    coils[1].set_pwm(0)
    coils[2].set_pwm(pwm)
    coils[3].set_pwm(-pwm)
    time.sleep(time_val)
    coils[0].set_pwm(-pwm)
    coils[1].set_pwm(pwm)
    coils[2].set_pwm(0)
    coils[3].set_pwm(0)
    time.sleep(time_val)
    coils[0].set_pwm(0)
    coils[1].set_pwm(0)
    coils[2].set_pwm(-pwm)
    coils[3].set_pwm(pwm)
    time.sleep(time_val)