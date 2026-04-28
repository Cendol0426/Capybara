#Develop Dashboard for manual control and tuning of locomotion parameters on the fly
#Step 1 Basic movement functions and PID control for straight movement and accurate turns
#Step 2 Command handler for Rpi communication and tuning parameters on the fly
#Step 3 Command status and data update for Rpi feedback and debugging
#Step 4 Development of control panel UI on Rpi for manual control and tuning

import tkinter as tk
import threading
import serial
import time

