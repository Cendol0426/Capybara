#Develop Dashboard for manual control and tuning of locomotion parameters on the fly
#Step 1 Basic movement functions and PID control for straight movement and accurate turns
#Step 2 Command handler for Rpi communication and tuning parameters on the fly
#Step 3 Command status and data update for Rpi feedback and debugging
#Step 4 Development of control panel UI on Rpi for manual control and tuning

import tkinter as tk
import threading
import serial
import time

#Serial Setup
ser = serial.Serial('COM5', 115200, timeout=1)  # change COM port
time.sleep(2)

#UI Setup
root =  tk.Tk()
root.title("Control Dashboard") #Set window title

#UI Arduino Output Display
output = tk.Text(root, height=10)
output.pack(fill="both", padx=10, pady=5) #Text box to display Arduino output 

def log(msg): #Log messages to the output box
    output.insert(tk.END, msg + "\n")
    output.see(tk.END)

#Communication Function
def send(cmd): #Send command to Arduino and log it
    ser.write((cmd + '\n').encode())
    log(">> " + cmd)

def send_tune(key, value): # Send tuning parameter updates to Arduino
    send(f"T:{key}={value}")

def get_value(): #Get distance/angle value from input, default to 0 if invalid
    try:
        return float(dist_entry.get())
    except:
        return 0

#Serial Reading Thread
def read_serial():
    while True:
        if ser.in_waiting:
            msg = ser.readline().decode(errors='ignore').strip()
            log(msg)

# Start thread to read serial data without blocking UI
threading.Thread(target=read_serial, daemon=True).start()

# Movement Control Section
control_frame = tk.LabelFrame(root, text="Movement Control", padx=10, pady=10)
control_frame.pack(fill="x", padx=10, pady=5)
tk.Label(control_frame, text="Distance / Angle").pack()

#Number Input
tk.Label(root, text="Distance / Angle").pack()
dist_entry = tk.Entry(root) #Text entry for numbers
dist_entry.pack() 
dist_entry.insert(0, "50")  # Default distance

#UI Movement Buttons
tk.Button(root, text="Forward", command=lambda: send(f"M:FWD:{get_value()}")).pack()

tk.Button(root, text="Backward", command=lambda: send(f"M:BWD:{get_value()}")).pack()

tk.Button(root, text="Turn Right", command=lambda: send(f"M:TR:{get_value()}")).pack()

tk.Button(root, text="Turn Left", command=lambda: send(f"M:TL:{get_value()}")).pack()

tk.Button(root, text="STOP", fg="red", command=lambda: send("M:STOP:0")).pack()

# Tuning Constants Section
tune_frame = tk.LabelFrame(root, text="Tuning Constants", padx=10, pady=10)
tune_frame.pack(fill="x", padx=10, pady=5)

#Base Speed Tuning
tk.Label(tune_frame, text="Base Speed").pack()
base_speed = tk.Scale(tune_frame, from_=0, to=255, orient='horizontal', command=lambda v: send_tune("BS", v))
base_speed.set(150)
base_speed.pack(fill="x")

#Min Speed Tuning
tk.Label(tune_frame, text="Min Speed").pack()
min_speed = tk.Scale(tune_frame, from_=0, to=255, orient='horizontal', command=lambda v: send_tune("MS", v))
min_speed.set(80)
min_speed.pack(fill="x")

#Turn Speed Tuning
tk.Label(tune_frame, text="Turn Speed").pack()
turn_speed = tk.Scale(tune_frame, from_=0, to=255, orient='horizontal', command=lambda v: send_tune("TS", v))
turn_speed.set(120)
turn_speed.pack(fill="x")

#Distance Calibration Tuning
#Ticks per cm Tuning
tk.Label(tune_frame, text="Ticks per cm").pack()
ticks_per_cm = tk.Scale(tune_frame, from_=0, to=100, orient='horizontal', command=lambda v: send_tune("TC", v))
ticks_per_cm.set(10)
ticks_per_cm.pack(fill="x")

#Ticks per degree Tuning
tk.Label(tune_frame, text="Ticks per degree").pack()
ticks_per_degree = tk.Scale(tune_frame, from_=0, to=100, orient='horizontal', command=lambda v: send_tune("TD", v))
ticks_per_degree.set(5)
ticks_per_degree.pack(fill="x")

#Deceleration Range
tk.Label(tune_frame, text="Deceleration Range").pack()
decel_range = tk.Scale(tune_frame, from_=0, to=100, orient='horizontal', command=lambda v: send_tune("DR", v))
decel_range.set(20)
decel_range.pack(fill="x")

#Angle Degree to Slow Down
tk.Label(tune_frame, text="Angle to Slow Down").pack()
angle_slow = tk.Scale(tune_frame, from_=0, to=180, orient='horizontal', command=lambda v: send_tune("TW", v))
angle_slow.set(90)
angle_slow.pack(fill="x")


root.mainloop()
