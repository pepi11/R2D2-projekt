import tkinter as tk
from websocket import WebSocketApp
import threading
import re
import time

# === Zmienne globalne ===
ws_global = None
rc_mode = False

# === Funkcje pomocnicze ===

def log(text):
    log_box.config(state="normal")
    log_box.insert("end", text + "\n")
    log_box.see("end")  # scroll to bottom
    log_box.config(state="disabled")

# === WebSocket handlers ===

def on_message(ws, message):
    if message.startswith("#DATA:"):
        match = re.search(r"DIST_K=(\d+);DIST=(\d+);YAW=([\d\.]+);V=([\d\.]+);", message)
        if match:
            dist_k = match.group(1)  # Kopułka
            dist = match.group(2)    # Gondola
            yaw = match.group(3)
            voltage = match.group(4)
            update_gui(dist_k, dist, yaw, voltage)
            log(f"[DATA] Kopułka={dist_k} mm, Gondola={dist} mm, YAW={yaw}, V={voltage}")
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

# === GUI ===

def update_gui(dist_k, dist, yaw, voltage):
    kopulka_var.set(dist_k + " mm")
    gondola_var.set(dist + " mm")
    yaw_var.set(yaw + "°")
    voltage_var.set(voltage + " V")

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

# === Tworzenie okna ===

root = tk.Tk()
root.title("R2D2 Telemetria")
root.geometry("350x520")
root.resizable(False, False)

tk.Label(root, text="Kopułka:", font=("Arial", 14)).pack(pady=2)
kopulka_var = tk.StringVar()
tk.Label(root, textvariable=kopulka_var, font=("Arial", 14)).pack()

tk.Label(root, text="Gondola:", font=("Arial", 14)).pack(pady=2)
gondola_var = tk.StringVar()
tk.Label(root, textvariable=gondola_var, font=("Arial", 14)).pack()

tk.Label(root, text="YAW:", font=("Arial", 14)).pack(pady=2)
yaw_var = tk.StringVar()
tk.Label(root, textvariable=yaw_var, font=("Arial", 14)).pack()

tk.Label(root, text="VOLTAGE:", font=("Arial", 14)).pack(pady=2)
voltage_var = tk.StringVar()
tk.Label(root, textvariable=voltage_var, font=("Arial", 14)).pack()

rc_button = tk.Button(root, text="RC_ON", font=("Arial", 12), command=toggle_rc_mode)
rc_button.pack(pady=10)

# === Pole logów ===
tk.Label(root, text="Logi:", font=("Arial", 12, "bold")).pack()
log_box = tk.Text(root, height=10, state="disabled", bg="#111", fg="#0f0", font=("Courier", 10))
log_box.pack(padx=10, pady=5, fill="both")

# === Start WebSocket w osobnym wątku ===
ws_thread = threading.Thread(target=start_websocket)
ws_thread.daemon = True
ws_thread.start()

root.mainloop()
