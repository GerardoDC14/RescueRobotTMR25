#!/usr/bin/env python3
import threading
import serial
import customtkinter as ctk

ctk.set_appearance_mode("dark")

# ─── GUI ────────────────────────────────────────────────────────────────
root = ctk.CTk()
root.title("PPM → Arm + Gripper")

ppm_ser = arm_ser = None
run_ppm = run_arm = moving = False

base_var     = ctk.StringVar(master=root, value="0.00")
shoulder_var = ctk.StringVar(master=root, value="0.00")
elbow_var    = ctk.StringVar(master=root, value="1.57")
hand_var     = ctk.StringVar(master=root, value="3.14")
ppm_stat     = ctk.StringVar(master=root, value="PPM: Disconnected")
arm_stat     = ctk.StringVar(master=root, value="Arm: Disconnected")

spd_var   = ctk.StringVar(master=root, value="0")
acc_var   = ctk.StringVar(master=root, value="10")

def send_to_arm(base, shoulder, elbow, hand):
    global arm_ser
    if not (run_arm and moving):
        return

    # Leer velocidad y aceleración
    try:
        spd = int(spd_var.get())
    except ValueError:
        spd = 0
    try:
        acc = int(acc_var.get())
    except ValueError:
        acc = 10

    cmd = {
        "T": 102,
        "base":      base,
        "shoulder":  shoulder,
        "elbow":     elbow,
        "hand":      hand,
        "spd":       spd,
        "acc":       acc
    }
    payload = json.dumps(cmd)
    arm_ser.write((payload + "\n").encode())
    print(">>> Forwarded to ARM:", payload)

def ppm_loop(port, baud):
    global ppm_ser, run_ppm
    try:
        ppm_ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        ppm_stat.set("PPM: Error")
        print("PPM open error:", e)
        return
    run_ppm = True
    ppm_stat.set("PPM: Connected")
    while run_ppm:
        line = ppm_ser.readline().decode(errors='ignore').strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) != 4:
            continue
        try:
            base     = float(parts[0])
            shoulder = float(parts[1])
            elbow    = float(parts[2])
            hand     = float(parts[3])
        except ValueError:
            continue

        # Actualiza GUI
        base_var.set(f"{base:.2f}")
        shoulder_var.set(f"{shoulder:.2f}")
        elbow_var.set(f"{elbow:.2f}")
        hand_var.set(f"{hand:.2f}")

        send_to_arm(base, shoulder, elbow, hand)

def connect_ppm():
    port = ppm_entry.get()
    baud = int(ppm_baud.get())
    threading.Thread(target=ppm_loop, args=(port, baud), daemon=True).start()

def connect_arm():
    global arm_ser, run_arm
    port = arm_entry.get()
    baud = int(arm_baud.get())
    try:
        arm_ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        arm_stat.set("Arm: Error")
        print("ARM open error:", e)
        return
    run_arm = True
    arm_stat.set("Arm: Connected")

def toggle_move():
    global moving
    moving = not moving
    move_btn.configure(text="Moving: ON" if moving else "Moving: OFF")

# ─── GUI ─────────────────────────────────────────────────────────────────
import json  

ctk.CTkLabel(root, textvariable=ppm_stat).grid(row=0, column=0, columnspan=2)
ctk.CTkLabel(root, text="PPM Port:").grid(row=1, column=0, sticky="e")
ppm_entry = ctk.CTkEntry(root); ppm_entry.grid(row=1, column=1)
ppm_baud = ctk.StringVar(value="115200")
ctk.CTkEntry(root, textvariable=ppm_baud, width=80).grid(row=1, column=2)
ctk.CTkButton(root, text="Connect PPM", command=connect_ppm).grid(row=1, column=3)

ctk.CTkLabel(root, textvariable=arm_stat).grid(row=2, column=0, columnspan=2, pady=(10,0))
ctk.CTkLabel(root, text="Arm Port:").grid(row=3, column=0, sticky="e")
arm_entry = ctk.CTkEntry(root); arm_entry.grid(row=3, column=1)
arm_baud = ctk.StringVar(value="115200")
ctk.CTkEntry(root, textvariable=arm_baud, width=80).grid(row=3, column=2)
ctk.CTkButton(root, text="Connect Arm", command=connect_arm).grid(row=3, column=3)

move_btn = ctk.CTkButton(root, text="Moving: OFF", command=toggle_move)
move_btn.grid(row=4, column=0, columnspan=4, pady=10)

# Lectura
ctk.CTkLabel(root, text="Base (rad):").grid(row=5, column=0, sticky="e")
ctk.CTkLabel(root, textvariable=base_var).grid(row=5, column=1, sticky="w")
ctk.CTkLabel(root, text="Shoulder (rad):").grid(row=6, column=0, sticky="e")
ctk.CTkLabel(root, textvariable=shoulder_var).grid(row=6, column=1, sticky="w")
ctk.CTkLabel(root, text="Elbow (rad):").grid(row=7, column=0, sticky="e")
ctk.CTkLabel(root, textvariable=elbow_var).grid(row=7, column=1, sticky="w")
ctk.CTkLabel(root, text="Hand (rad):").grid(row=8, column=0, sticky="e")
ctk.CTkLabel(root, textvariable=hand_var).grid(row=8, column=1, sticky="w")

# Controles de velocidad y aceleración
ctk.CTkLabel(root, text="Speed:").grid(row=9, column=0, sticky="e", pady=(10,0))
spd_entry = ctk.CTkEntry(root, textvariable=spd_var, width=60); spd_entry.grid(row=9, column=1, pady=(10,0))
ctk.CTkLabel(root, text="Accel:").grid(row=9, column=2, sticky="e", pady=(10,0))
acc_entry = ctk.CTkEntry(root, textvariable=acc_var, width=60); acc_entry.grid(row=9, column=3, pady=(10,0))

root.mainloop()
