import socket
import json
import threading
import time
import numpy as np

# ======== OSC ========
OSC_ENABLE = True
TD_IP = "127.0.0.1"
OSC_PORT = 7000

osc_client = None
if OSC_ENABLE:
    try:
        from pythonosc.udp_client import SimpleUDPClient
        osc_client = SimpleUDPClient(TD_IP, OSC_PORT)
        print(f"[OSC] Ready to send to {TD_IP}:{OSC_PORT}")
    except ImportError:
        print("[OSC] Please install python-osc: pip install python-osc")
        OSC_ENABLE = False
    except Exception as e:
        print(f"[OSC] Initialization error: {e}")
        OSC_ENABLE = False

# ++++++++ ใหม่: SciPy (สำหรับ Iterative Solver) ++++++++
try:
    import scipy.optimize
except ImportError:
    print("[Solver] โปรดติดตั้ง SciPy: pip install scipy")
    # ปิดการใช้งานตัวแก้ปัญหาแบบวนซ้ำถ้าไม่มี SciPy
    # (ในตัวอย่างนี้ เราจะปล่อยให้มัน crash ถ้าไม่มี แต่คุณสามารถเพิ่มตรรกะสำรองได้)
    pass

# ======== Server Settings ========
HOST = "0.0.0.0"
PORT = 3334

# ======== Anchors (cm) ========
# A0:(0,0,250)  A1:(500,0,250)  A2:(0,0,0)  A3:(250,250,250)
ANCHORS = [
    ("A0", 0.0,   0.0,   200.0),
    ("A1", 370.0, 0.0,   0.0),
    ("A2", 0.0,   290.0, 20.0),
    ("A3", 370.0, 290.0, 200.0),  # reference
]
# สร้างเวอร์ชัน numpy ที่ใช้บ่อยไว้ล่วงหน้า
ANCHORS_NP = np.array([[ax, ay, az] for (_, ax, ay, az) in ANCHORS], dtype=float)

# ======== Kalman Filter Settings (ใหม่) ========
# ปรับค่า Q และ R เพื่อจูนฟิลเตอร์:
# Q: Process Noise - ความไม่แน่นอนของโมเดล (ยิ่งน้อย ยิ่ง "หนืด" หรือเคลื่อนที่ช้า)
# R: Measurement Noise - ความไม่แน่นอนของค่าที่วัดได้จาก UWB (ยิ่งมาก ยิ่ง "ไม่เชื่อ" ค่าที่โดดๆ)
KALMAN_Q = 1.0  # (cm^2) - ลองปรับค่านี้ระหว่าง 0.01 ถึง 1.0
KALMAN_R = 15.0 # (cm^2) - ลองปรับค่านี้ระหว่าง 5.0 ถึง 50.0

# ======== Bounds (cm) ========
X_MIN, X_MAX = 50.0, 400.0
Y_MIN, Y_MAX = 50.0, 290.0
Z_MIN, Z_MAX = 50.0, 200.0

def clamp(v, lo, hi):  # <<— helper
    return lo if v < lo else (hi if v > hi else v)

# สถานะต่อแท็ก: tag_id -> {"kf": KalmanFilter3D, "last_good_pos": [x,y,z]}
tag_states = {}

# ======== (ใหม่) Kalman Filter 3D Class ========
class KalmanFilter3D:
    """
    ฟิลเตอร์ Kalman 3D แบบง่าย (โมเดลความเร็วคงที่ = 0)
    F = I, H = I
    """
    def __init__(self, initial_state, q_noise=0.1, r_noise=10.0):
        # สถานะ [x, y, z]
        self.x = np.array(initial_state, dtype=float).reshape(3, 1)
        # Covariance ของสถานะ
        self.P = np.eye(3) * 10.0
        # Process Noise Covariance
        self.Q = np.eye(3) * q_noise
        # Measurement Noise Covariance
        self.R = np.eye(3) * r_noise
        # (F และ H เป็น Identity matrix (np.eye(3)) ในโมเดลนี้)

    def predict(self):
        # x_k = F * x_{k-1}  (F=I, ดังนั้น x_k = x_{k-1})
        # P_k = F * P_{k-1} * F^T + Q  (F=I, ดังนั้น P_k = P_{k-1} + Q)
        self.P = self.P + self.Q
        return self.x.flatten() # คืนค่าที่ predict ไว้ก่อน

    def update(self, measurement):
        Z = np.array(measurement, dtype=float).reshape(3, 1)

        # y = Z - H * x  (H=I)
        y = Z - self.x
        # S = H * P * H^T + R  (H=I)
        S = self.P + self.R
        # K = P * H^T * S_inv  (H=I)
        K = self.P @ np.linalg.inv(S)

        # x_new = x + K * y
        self.x = self.x + (K @ y)
        # P_new = (I - K * H) * P  (H=I)
        self.P = (np.eye(3) - K) @ self.P

        return self.x.flatten() # คืนค่าที่ update แล้ว

# ======== (เก่า) Solver: AX=b (A3 เป็นอ้างอิง) - (เปลี่ยนชื่อ) ========
def get_initial_guess_linear(anchors_named, ranges_cm, ref_idx=3):
    """
    ใช้วิธี Linear Least Squares (แบบเก่า) เพื่อหา "ค่าเดาเริ่มต้น"
    """
    a = np.array([[ax, ay, az] for (_, ax, ay, az) in anchors_named], dtype=float)
    d = np.array(ranges_cm[:4], dtype=float)
    if a.shape[0] < 4 or d.shape[0] < 4 or np.any(np.isnan(d)):
        return None
    aref = a[ref_idx]; dref = d[ref_idx]
    rows, rhs = [], []
    for i in range(4):
        if i == ref_idx:
            continue
        Ai = 2.0 * (aref - a[i])
        bi = (aref @ aref) - (a[i] @ a[i]) + (d[i]**2 - dref**2)
        rows.append(Ai); rhs.append(bi)
    A = np.vstack(rows); b = np.array(rhs)
    if np.linalg.matrix_rank(A) < 3:
        return None
    try:
        X = np.linalg.solve(A, b)
        return float(X[0]), float(X[1]), float(X[2])
    except np.linalg.LinAlgError:
        return None


# ======== (ใหม่) Solver: Iterative Non-Linear Least Squares ========
def residual_func(X, anchors, ranges):
    """
    ฟังก์ชันคำนวณ "ส่วนต่าง" (residual)
    X: [x, y, z] ตำแหน่งที่กำลังทดสอบ
    anchors: (N, 3) array ของตำแหน่ง anchor
    ranges: (N,) array ของระยะทางที่วัดได้
    """
    # คำนวณระยะทางแบบ Euclidean จาก X ไปยัง anchor ทั้งหมด
    calculated_distances = np.linalg.norm(anchors - X, axis=1)
    # คืนค่า (ระยะที่วัดได้ - ระยะที่คำนวณได้)
    return ranges - calculated_distances

def solve_3d_iterative_LS(anchors_np, ranges_cm, initial_guess):
    """
    แก้ปัญหา 3D Trilateration โดยใช้ Iterative Non-Linear Least Squares
    (ผ่าน scipy.optimize.least_squares)
    """
    ranges_np = np.array(ranges_cm, dtype=float)
    
    try:
        # ใช้ 'lm' (Levenberg-Marquardt) ซึ่งทนทานและมีประสิทธิภาพสูง
        result = scipy.optimize.least_squares(
            residual_func,
            initial_guess,
            args=(anchors_np, ranges_np),
            method='lm'
        )
        
        if result.success:
            return result.x # คืนค่า [x, y, z]
        else:
            return None
    except Exception as e:
        print(f"[Solver] Iterative LS Error: {e}")
        return None

# ======== Utils: ส่ง OSC ต่อแท็ก ========
def osc_send_tag(tag_id, x, y, z):
    if not (OSC_ENABLE and osc_client):
        return
    base = f"/uwb/tag/{tag_id}"
    #osc_client.send_message(base + "/pos",  [x, y])           # 2D legacy
    osc_client.send_message(base + "/pos3", [x, y, z])        # 3D

# ======== Server Thread (ปรับปรุงใหม่) ========
def server_thread():
    """
    รับ JSON ต่อบรรทัด:
      {"tag":0,"ranges_cm":[d0,d1,d2,d3], ...}
      {"tag":1,"ranges_cm":[d0,d1,d2,d3], ...}
    แล้วคำนวณ 3D ต่อแท็ก
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"Listening for UWB data on {HOST}:{PORT} ...")

        while True:
            conn, addr = s.accept()
            print(f"Client connected: {addr}")
            try:
                buf = b""
                while True:
                    chunk = conn.recv(2048)
                    if not chunk:
                        print("Client disconnected.")
                        break

                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            msg = json.loads(line.decode("utf-8"))

                            # ต้องมี tag และ ranges_cm>=4
                            if ("tag_id" in msg or "tag" in msg) and "ranges_cm" in msg:
                                tag_id = int(msg.get("tag_id", msg.get("tag")))
                                ranges = msg["ranges_cm"]
                                if not (isinstance(ranges, list) and len(ranges) >= 4):
                                    continue

                                # แปลงเป็น float และตรวจ NaN
                                try:
                                    r = [float(x) if x is not None else np.nan for x in ranges[:4]]
                                except Exception:
                                    continue
                                if any(np.isnan(r)):
                                    print(f"[Tag {tag_id}] ข้อมูลระยะทางไม่ถูกต้อง (NaN)")
                                    continue
                                
                                # --- (ใหม่) ตรรกะการคำนวณและกรอง ---

                                # 1. หาค่าเดาเริ่มต้น (Initial Guess)
                                if tag_id in tag_states:
                                    # ถ้าเคยเห็นแท็กนี้แล้ว ใช้ตำแหน่งล่าสุด (ที่กรองแล้ว) เป็นค่าเดา
                                    guess = tag_states[tag_id]["last_good_pos"]
                                else:
                                    # ถ้าเป็นแท็กใหม่ ใช้ Solver แบบ Linear (ตัวเก่า) เพื่อหาค่าเดา
                                    guess_linear = get_initial_guess_linear(ANCHORS, r, ref_idx=3)
                                    if guess_linear:
                                        guess = guess_linear
                                    else:
                                        # ถ้า Solver เก่าล้มเหลว (เช่น rankไม่พอ) ใช้จุดกึ่งกลางห้อง (โดยประมาณ)
                                        guess = [200.0, 100.0, 100.0]
                                    
                                    # สร้าง Kalman Filter ใหม่สำหรับแท็กนี้
                                    tag_states[tag_id] = {
                                        "kf": KalmanFilter3D(guess, q_noise=KALMAN_Q, r_noise=KALMAN_R),
                                        "last_good_pos": guess
                                    }

                                # 2. คำนวณตำแหน่ง (Measurement) ด้วย Solver ใหม่ (Iterative LS)
                                p_measured = solve_3d_iterative_LS(ANCHORS_NP, r, guess)
                                
                                if p_measured is None:
                                    print(f"[Tag {tag_id}] Solver (Iterative) ล้มเหลว")
                                    # ถ้าล้มเหลว ใช้ค่า predict ของ Kalman แทน (เพื่อให้เคลื่อนที่ต่อเนียนๆ)
                                    p_filtered = tag_states[tag_id]["kf"].predict()
                                    
                                    # 4. บันทึกสถานะ และส่งข้อมูล
                                    x, y, z = p_filtered
                                    tag_states[tag_id]["last_good_pos"] = [x, y, z] # เก็บค่าที่กรองแล้วไว้ใช้เป็น guess รอบหน้า
                                    
                                    print(f"[Tag {tag_id}] (X,Y,Z) = ({x:.2f}, {y:.2f}, {z:.2f}) cm [Raw: Solver Failed]")
                                    osc_send_tag(tag_id, x, y, z)
                                else:
                                    # 3. กรองสัญญาณด้วย Kalman Filter
                                    kf = tag_states[tag_id]["kf"]
                                    kf.predict() # คาดการณ์ว่าจะไปไหน
                                    p_filtered = kf.update(p_measured) # อัปเดตด้วยค่าที่วัดได้
                                
                                    # 4. บันทึกสถานะ และส่งข้อมูล
                                    x, y, z = p_filtered
                                    tag_states[tag_id]["last_good_pos"] = [x, y, z] # เก็บค่าที่กรองแล้วไว้ใช้เป็น guess รอบหน้า

                                    # ---- Clamp ให้อยู่ในกรอบ ----
                                    x = clamp(x, X_MIN, X_MAX)
                                    y = clamp(y, Y_MIN, Y_MAX)
                                    z = clamp(z, Z_MIN, Z_MAX)

                                    print(f"[Tag {tag_id}] (X,Y,Z) = ({x:.2f}, {y:.2f}, {z:.2f}) cm [Raw: ({p_measured[0]:.2f}, {p_measured[1]:.2f}, {p_measured[2]:.2f}) cm]")
                                    osc_send_tag(tag_id, x, y, z)

                        except json.JSONDecodeError:
                            # ถ้าบอร์ดส่งเป็นข้อความ AT เดิม (ไม่ใช่ JSON) ให้แปลงเป็น JSON ที่บอร์ดก่อน
                            # ตรงนี้ตั้งใจข้ามเพื่อให้ระบบแข็งแรงและบังคับรูปแบบเดียว
                            pass
                        except Exception as e:
                            print(f"Error processing line: {e}")

            except (ConnectionResetError, BrokenPipeError) as e:
                print(f"Connection error: {e}")
            finally:
                conn.close()
                print("Connection closed. Waiting for new client...")

# ======== Main ========
if __name__ == "__main__":
    threading.Thread(target=server_thread, daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")