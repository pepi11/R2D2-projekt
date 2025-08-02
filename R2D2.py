import tkinter as tk
from websocket import WebSocketApp
import threading
import re
import time

# === KONFIGURACJA ===
ws_global = None
rc_mode = False

# === Funkcje pomocnicze ===
def log(text):
    log_box.config(state="normal")
    log_box.insert("end", text + "\n")
    log_box.see("end")
    log_box.config(state="disabled")

# === WebSocket handlers ===
def on_message(ws, message):
    if message.startswith("#DATA:"):
        match = re.search(r"DIST_K=(\d+);DIST=(\d+);YAW=([\d\.]+);V=([\d\.]+);I=([\d\.]+);", message)
        if match:
            dist_k = match.group(1)
            dist = match.group(2)
            yaw = match.group(3)
            voltage = match.group(4)
            current = match.group(5)
            update_gui(dist_k, dist, yaw, voltage, current)
            log(f"[DATA] Kopułka={dist_k} mm, Gondola={dist} mm, YAW={yaw}, V={voltage}, I={current} mA")
    else:
        log(f"[MSG] {message}")

def on_open(ws):
    log("🔗 Połączono z ESP32")
    ws.send("R2D2:PLAY:4:15")

def on_close(ws, close_status_code, close_msg):
    log("🔌 Połączenie zamknięte")

def on_error(ws, error):
    log(f"❌ Błąd: {error}")

def start_websocket():
    global ws_global
    while True:
        try:
            ws_global = WebSocketApp("ws://192.168.0.163:81",
                                     on_message=on_message,
                                     on_open=on_open,
                                     on_close=on_close,
                                     on_error=on_error)
            ws_global.run_forever()
        except Exception as e:
            log(f"⚠️ Błąd ogólny: {e}")
        time.sleep(5)

# === GUI Telemetrii ===
def update_gui(dist_k, dist, yaw, voltage, current):
    kopulka_var.set(dist_k + " mm")
    gondola_var.set(dist + " mm")
    yaw_var.set(yaw + "°")
    voltage_var.set(voltage + " V")
    current_var.set(current + " mA")

def toggle_rc_mode():
    global rc_mode, ws_global
    if not ws_global or not ws_global.sock or not ws_global.sock.connected:
        log("❌ WebSocket niegotowy")
        return
    if rc_mode:
        ws_global.send("#CMD:RC_OFF;")
        rc_button.config(text="RC_ON")
        log("📴 Tryb RC wyłączony")
    else:
        ws_global.send("#CMD:RC_ON;")
        rc_button.config(text="RC_OFF")
        log("📡 Tryb RC włączony")
    rc_mode = not rc_mode

def send_move(forward=True):
    try:
        dist = float(entry_dist.get())
        cmd = f"#CMD:MOVE:{dist:.1f}:{0 if forward else 1};"
        ws_global.send(cmd)
        log(f"📤 Wysłano: {cmd}")
    except ValueError:
        log("❌ Nieprawidłowa wartość odległości")

def send_turn(left=True):
    try:
        angle = float(entry_kat.get())
        cmd = f"#CMD:TURN:{angle:.1f}:{1 if left else 0};"
        ws_global.send(cmd)
        log(f"📤 Wysłano: {cmd}")
    except ValueError:
        log("❌ Nieprawidłowa wartość kąta")

# === Tworzenie GUI ===
root = tk.Tk()
root.title("R2D2 – Telemetria + Sterowanie")
root.geometry("500x850")
root.resizable(False, False)

main_frame = tk.Frame(root)
main_frame.pack(pady=10, padx=10)

left_frame = tk.Frame(main_frame)
left_frame.pack(side="left", padx=10)

right_frame = tk.Frame(main_frame)
right_frame.pack(side="left", padx=10)

# Telemetria
tk.Label(left_frame, text="Kopułka:", font=("Arial", 14)).pack(pady=2)
kopulka_var = tk.StringVar()
tk.Label(left_frame, textvariable=kopulka_var, font=("Arial", 14)).pack()

tk.Label(left_frame, text="Gondola:", font=("Arial", 14)).pack(pady=2)
gondola_var = tk.StringVar()
tk.Label(left_frame, textvariable=gondola_var, font=("Arial", 14)).pack()

tk.Label(left_frame, text="AZYMUT:", font=("Arial", 14)).pack(pady=2)
yaw_var = tk.StringVar()
tk.Label(left_frame, textvariable=yaw_var, font=("Arial", 14)).pack()

tk.Label(left_frame, text="NAPIĘCIE:", font=("Arial", 14)).pack(pady=2)
voltage_var = tk.StringVar()
tk.Label(left_frame, textvariable=voltage_var, font=("Arial", 14)).pack()

tk.Label(left_frame, text="PRĄD:", font=("Arial", 14)).pack(pady=2)
current_var = tk.StringVar()
tk.Label(left_frame, textvariable=current_var, font=("Arial", 14)).pack()

rc_button = tk.Button(left_frame, text="RC_ON", font=("Arial", 12), command=toggle_rc_mode)
rc_button.pack(pady=10)

# Panel sterowania
tk.Label(right_frame, text="Odległość (cm):", font=("Arial", 10)).pack()
entry_dist = tk.Entry(right_frame, width=10, font=("Arial", 12))
entry_dist.insert(0, "20")
entry_dist.pack(pady=2)

tk.Label(right_frame, text="Kąt (°):", font=("Arial", 10)).pack()
entry_kat = tk.Entry(right_frame, width=10, font=("Arial", 12))
entry_kat.insert(0, "90")
entry_kat.pack(pady=2)

pad_frame = tk.Frame(right_frame)
pad_frame.pack(pady=10)

btn_up = tk.Button(pad_frame, text="↑", width=4, height=2, command=lambda: send_move(True))
btn_up.grid(row=0, column=1)

btn_left = tk.Button(pad_frame, text="←", width=4, height=2, command=lambda: send_turn(True))
btn_left.grid(row=1, column=0)

btn_right = tk.Button(pad_frame, text="→", width=4, height=2, command=lambda: send_turn(False))
btn_right.grid(row=1, column=2)

btn_down = tk.Button(pad_frame, text="↓", width=4, height=2, command=lambda: send_move(False))
btn_down.grid(row=2, column=1)

# Logi
tk.Label(root, text="Logi:", font=("Arial", 12, "bold")).pack()
log_box = tk.Text(root, height=26, state="disabled", bg="#111", fg="#0f0", font=("Courier", 10))
log_box.pack(padx=10, pady=5, fill="x")

# WebSocket
ws_thread = threading.Thread(target=start_websocket)
ws_thread.daemon = True
ws_thread.start()

root.mainloop()