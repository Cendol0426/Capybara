#Develop Dashboard for manual control and tuning of locomotion parameters on the fly
#Step 1 Basic movement functions and PID control for straight movement and accurate turns
#Step 2 Command handler for Rpi communication and tuning parameters on the fly
#Step 3 Command status and data update for Rpi feedback and debugging
#Step 4 Development of control panel UI on Rpi for manual control and tuning

import tkinter as tk
import threading
import serial
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import collections
from queue import Queue  # ============= MODIFIED / ADDED =============

#Serial Setup
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)

#UI Setup
root =  tk.Tk()
root.title("Control Dashboard")

# Thread-safe queue for graph + UI updates
data_queue = Queue()

#UI Arduino Output Display
output = tk.Text(root, height=10)
output.pack(fill="both", padx=10, pady=5)

def log(msg):
    output.insert(tk.END, msg + "\n")
    output.see(tk.END)

#Communication Function
def send(cmd):
    ser.write((cmd + '\n').encode())
    log(">> " + cmd)

def send_tune(key, value):
    send(f"T:{key}={value}")

def get_value():
    try:
        return float(dist_entry.get())
    except:
        return 0

# Serial thread ONLY pushes data to queue (no UI calls!)
def read_serial():
    while True:
        if ser.in_waiting:
            msg = ser.readline().decode(errors='ignore').strip()
            data_queue.put(msg)

threading.Thread(target=read_serial, daemon=True).start()

# UI-safe updater (runs in main thread)
def process_queue():
    while not data_queue.empty():
        msg = data_queue.get()
        log(msg)

        try:
            if "ERR:" in msg:
                val = float(msg.split(":")[1])
                update_graph(error=val)

            elif "SPD:" in msg:
                val = float(msg.split(":")[1])
                update_graph(speed=val)
        except:
            pass

    root.after(50, process_queue)  # refresh every 50ms

# Movement Control Section
control_frame = tk.LabelFrame(root, text="Movement Control", padx=10, pady=10)
control_frame.pack(fill="x", padx=10, pady=5)

tk.Label(control_frame, text="Distance / Angle").pack()

tk.Label(root, text="Distance / Angle").pack()
dist_entry = tk.Entry(root)
dist_entry.pack()
dist_entry.insert(0, "50")

tk.Button(root, text="Forward", command=lambda: send(f"M:FWD:{get_value()}")).pack()
tk.Button(root, text="Backward", command=lambda: send(f"M:BWD:{get_value()}")).pack()
tk.Button(root, text="Turn Right", command=lambda: send(f"M:TR:{get_value()}")).pack()
tk.Button(root, text="Turn Left", command=lambda: send(f"M:TL:{get_value()}")).pack()
tk.Button(root, text="STOP", fg="red", command=lambda: send("M:STOP:0")).pack()

# Tuning Constants Section
tune_frame = tk.LabelFrame(root, text="Tuning Constants", padx=10, pady=10)
tune_frame.pack(fill="x", padx=10, pady=5)

#Base Speed
tk.Label(tune_frame, text="Base Speed").pack()
base_speed = tk.Scale(tune_frame, from_=0, to=255, orient='horizontal', command=lambda v: send_tune("BS", v))
base_speed.set(150)
base_speed.pack(fill="x")

#Min Speed
tk.Label(tune_frame, text="Min Speed").pack()
min_speed = tk.Scale(tune_frame, from_=0, to=255, orient='horizontal', command=lambda v: send_tune("MS", v))
min_speed.set(80)
min_speed.pack(fill="x")

#Turn Speed
tk.Label(tune_frame, text="Turn Speed").pack()
turn_speed = tk.Scale(tune_frame, from_=0, to=255, orient='horizontal', command=lambda v: send_tune("TS", v))
turn_speed.set(120)
turn_speed.pack(fill="x")

#Ticks per cm
tk.Label(tune_frame, text="Ticks per cm").pack()
ticks_per_cm = tk.Scale(tune_frame, from_=0, to=100, orient='horizontal', command=lambda v: send_tune("TC", v))
ticks_per_cm.set(10)
ticks_per_cm.pack(fill="x")

#Ticks per degree
tk.Label(tune_frame, text="Ticks per degree").pack()
ticks_per_degree = tk.Scale(tune_frame, from_=0, to=100, orient='horizontal', command=lambda v: send_tune("TD", v))
ticks_per_degree.set(5)
ticks_per_degree.pack(fill="x")

#Deceleration Range
tk.Label(tune_frame, text="Deceleration Range").pack()
decel_range = tk.Scale(tune_frame, from_=0, to=100, orient='horizontal', command=lambda v: send_tune("DR", v))
decel_range.set(20)
decel_range.pack(fill="x")

#Angle to Slow Down
tk.Label(tune_frame, text="Angle to Slow Down").pack()
angle_slow = tk.Scale(tune_frame, from_=0, to=180, orient='horizontal', command=lambda v: send_tune("TW", v))
angle_slow.set(90)
angle_slow.pack(fill="x")

# PID Tuning Section
pid_frame = tk.LabelFrame(root, text="PID Tuning", padx=10, pady=10)
pid_frame.pack(fill="x", padx=10, pady=5)

def create_pid_controls(parent, title, key):
    frame = tk.LabelFrame(parent, text=title, padx=5, pady=5)
    frame.pack(fill="x", pady=5)

    tk.Label(frame, text="Kp").pack()
    kp = tk.Scale(frame, from_=0, to=10, resolution=0.1, orient='horizontal')
    kp.set(2.0)
    kp.pack(fill="x")

    tk.Label(frame, text="Ki").pack()
    ki = tk.Scale(frame, from_=0, to=5, resolution=0.01, orient='horizontal')
    ki.set(0.0)
    ki.pack(fill="x")

    tk.Label(frame, text="Kd").pack()
    kd = tk.Scale(frame, from_=0, to=5, resolution=0.1, orient='horizontal')
    kd.set(0.0)
    kd.pack(fill="x")

    def apply_pid():
        cmd = f"{kp.get()},{ki.get()},{kd.get()}"
        send_tune(key, cmd)

    tk.Button(frame, text="Apply", command=apply_pid).pack(pady=5)

create_pid_controls(pid_frame, "Straight PID", "SPID")
create_pid_controls(pid_frame, "Turn PID", "TPID")
create_pid_controls(pid_frame, "Balance PID", "BPID")

# Graph Section
graph_frame = tk.LabelFrame(root, text="PID Live Graph", padx=10, pady=10)
graph_frame.pack(fill="both", expand=True, padx=10, pady=5)

fig, ax = plt.subplots()
ax.set_title("Error + Speed")
ax.set_xlabel("Time")
ax.set_ylabel("Value")

data_x = collections.deque(maxlen=100)
error_y = collections.deque(maxlen=100)
speed_y = collections.deque(maxlen=100)

line_err, = ax.plot([], [], label="Error")
line_spd, = ax.plot([], [], label="Speed")

ax.legend()

canvas = FigureCanvasTkAgg(fig, master=graph_frame)
canvas.get_tk_widget().pack(fill="both", expand=True)

t = 0

def update_graph(error=None, speed=None):
    global t

    data_x.append(t)

    # Keep last value if None (prevents graph drop)
    if error is not None:
        error_y.append(error)
    else:
        error_y.append(error_y[-1] if error_y else 0)

    if speed is not None:
        speed_y.append(speed)
    else:
        speed_y.append(speed_y[-1] if speed_y else 0)

    t += 1

    line_err.set_data(data_x, error_y)
    line_spd.set_data(data_x, speed_y)

    ax.relim()
    ax.autoscale_view()
    canvas.draw_idle()  # smoother than draw()

# Start UI-safe update loop
process_queue()

root.mainloop()
