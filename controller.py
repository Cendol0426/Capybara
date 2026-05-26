from pyPS4Controller.controller import Controller
import serial
import time

try:
    arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2)
    print("Connected to Arduino.")
except Exception as e:
    print(f"Serial connection failed: {e}")
    exit()

class RobotController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        # Default value 128 is center
        self.ly = 128
        self.rx = 128
        self.btn_start = 0
        self.btn_select = 0
        self.btn_blue = 0
        self.btn_l2 = 0
        self.btn_r2 = 0
        self.btn_up = 0
        self.btn_down = 0

    def send_state(self):
        # Format: C:LY,RX,START,SELECT,BLUE,L2,R2,UP,DOWN
        cmd = f"C:{self.ly},{self.rx},{self.btn_start},{self.btn_select},{self.btn_blue},{self.btn_l2},{self.btn_r2},{self.btn_up},{self.btn_down}\n"
        arduino.write(cmd.encode('utf-8'))

    def map_joystick(self, value):
        # Map PS4 range (-32767 to 32767) to PS2 range (0 to 255)
        return int((value + 32767) / 65534.0 * 255)

    def on_L3_up(self, value):
        self.ly = self.map_joystick(value)
        self.send_state()

    def on_L3_down(self, value):
        self.ly = self.map_joystick(value)
        self.send_state()

    def on_L3_y_at_rest(self):
        self.ly = 128
        self.send_state()

    def on_R3_left(self, value):
        self.rx = self.map_joystick(value)
        self.send_state()

    def on_R3_right(self, value):
        self.rx = self.map_joystick(value)
        self.send_state()

    def on_R3_x_at_rest(self):
        self.rx = 128
        self.send_state()

    def on_options_press(self): # PS4 Options = PS2 Start
        self.btn_start = 1; self.send_state()
    def on_options_release(self):
        self.btn_start = 0; self.send_state()

    def on_share_press(self): # PS4 Share = PS2 Select
        self.btn_select = 1; self.send_state()
    def on_share_release(self):
        self.btn_select = 0; self.send_state()

    def on_x_press(self): # PS4 X = PS2 Blue
        self.btn_blue = 1; self.send_state()
    def on_x_release(self):
        self.btn_blue = 0; self.send_state()

    def on_L2_press(self, value):
        self.btn_l2 = 1; self.send_state()
    def on_L2_release(self):
        self.btn_l2 = 0; self.send_state()

    def on_R2_press(self, value):
        self.btn_r2 = 1; self.send_state()
    def on_R2_release(self):
        self.btn_r2 = 0; self.send_state()

    # D-Pad
    def on_up_arrow_press(self):
        self.btn_up = 1; self.send_state()
    def on_down_arrow_press(self):
        self.btn_down = 1; self.send_state()
    def on_up_down_arrow_release(self):
        self.btn_up = 0; self.btn_down = 0; self.send_state()

print("Waiting for PS4 controller input...")
controller = RobotController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()