import cv2
import numpy as np
import time
from serialm import SManage

PWM_LURUS = 70
PWM_BELOK = 60

MAX_ERROR = 220
BATAS_MEMORI = 5
BATAS_AMBANG_BELOK = 20

MIN_LINE_WIDTH = 10
MAX_LINE_WIDTH = 120
MIN_AREA = 100

riwayat_error = []
error_terakhir = 0
last_sent_angle = -1
last_sent_pwm = -1

def hitung_sudut_servo(error, max_error=MAX_ERROR, min_angle=45, max_angle=135):
    error_clamped = max(-max_error, min(max_error, error))
    angle = 90 + int((error_clamped / max_error) * 45)
    return max(min_angle, min(max_angle, angle))


def proses_jalur(frame):
    global riwayat_error, error_terakhir
    
    tinggi, lebar, _ = frame.shape
    tengah_layar = lebar // 2
    
    roi_mulai_y = int(tinggi * 0.6)  
    roi = frame[roi_mulai_y:tinggi, 0:lebar]
    
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 180])      
    upper_white = np.array([179, 45, 255])   
    masker = cv2.inRange(hsv, lower_white, upper_white)
    
    kernel = np.ones((3, 3), np.uint8)
    masker = cv2.morphologyEx(masker, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(masker, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    garis_valid = None
    lebar_garis_deteksi = 0
    
    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                
                if MIN_LINE_WIDTH <= w <= MAX_LINE_WIDTH:
                    garis_valid = cnt
                    lebar_garis_deteksi = w
                    
                    cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    break
                
                elif w > MAX_LINE_WIDTH:
                    cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(roi, "BLOK TERLALU BESAR!", (x, max(15, y - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    if garis_valid is not None:
        M = cv2.moments(garis_valid)
        if M["m00"] != 0:
            cx_asli = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            cx_target = max(0, cx_asli - 0)
            error_mentah = cx_target - tengah_layar
            
            riwayat_error.append(error_mentah)
            if len(riwayat_error) > BATAS_MEMORI:
                riwayat_error.pop(0)
            
            error_stabil = int(sum(riwayat_error) / len(riwayat_error))
            error_terakhir = error_stabil
            
            cv2.circle(roi, (cx_asli, cy), 5, (0, 0, 255), -1)
            cv2.circle(roi, (cx_target, cy), 7, (0, 255, 0), -1)
            status_garis = f"Terdeteksi (W:{lebar_garis_deteksi}px)"
            warna_teks = (0, 255, 0)
        else:
            error_stabil = error_terakhir
            status_garis = "HILANG! (Pakai Memori)"
            warna_teks = (0, 0, 255)
    else:
        error_stabil = error_terakhir
        status_garis = "ABAIKAN BLOK / GARIS HILANG"
        warna_teks = (0, 0, 255)
        
    servo_angle = hitung_sudut_servo(error_stabil)

    if abs(error_stabil) > BATAS_AMBANG_BELOK:
        pwm_target = PWM_BELOK 
        mode_gerak = f"BELOK (PWM {PWM_BELOK})"
    else:
        pwm_target = PWM_LURUS 
        mode_gerak = f"LURUS (PWM {PWM_LURUS})"

    cv2.putText(frame, f"Garis  : {status_garis}", (20, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, warna_teks, 2)
    cv2.putText(frame, f"Status : {mode_gerak}", (20, 55), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(frame, f"Servo  : {servo_angle} deg", (20, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.line(frame, (tengah_layar, 0), (tengah_layar, tinggi), (255, 0, 0), 2)
    
    return frame, masker, servo_angle, pwm_target

def kirim_data_serial(robot_ctrl, speed, angle):
    """Mengirim data dengan protokol baru 'speed,servo'."""
    global last_sent_angle, last_sent_pwm
    
    if abs(angle - last_sent_angle) >= 2 or speed != last_sent_pwm:
        paket_data = f"{speed},{angle}\n"
        
        if hasattr(robot_ctrl, 'send_raw'):
            robot_ctrl.send_raw(paket_data)
        elif hasattr(robot_ctrl, 'send'):
            robot_ctrl.send(paket_data)
            
        last_sent_angle = angle
        last_sent_pwm = speed


def main():
    robot_ctrl = SManage(port='/dev/ttyUSB0', baudrate=9600)

    if not robot_ctrl.connect():
        print("[!] Gagal terhubung ke ESP32. Berjalan dalam mode SIMULASI.")

    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG'))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("[X] Error: Kamera tidak dapat dibuka!")
        robot_ctrl.close()
        return

    print("\n==============================================")
    print("   ROBOT LINE TRACKER (FILTER BLOK PUTIH)")
    print("   Target Speed: Lurus = 70 | Belok = 60")
    print(f"   Limit Lebar Line Valid: {MIN_LINE_WIDTH}px - {MAX_LINE_WIDTH}px")
    print("   Tekan 'q' untuk berhenti.")
    print("==============================================\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[X] Kamera terputus!")
                break
                
            frame_hasil, visual_masker, target_angle, pwm_target = proses_jalur(frame)
            
            if robot_ctrl.is_connected():
                kirim_data_serial(robot_ctrl, pwm_target, target_angle)
            
            cv2.imshow("Kamera Utama Robot", frame_hasil)
            cv2.imshow("Masker Garis Putih", visual_masker)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[!] Dihentikan secara paksa oleh pengguna.")

    finally:
        print("\n[*] Mengirim perintah STOP ke ESP32...")
        if robot_ctrl.is_connected():
            if hasattr(robot_ctrl, 'send_raw'):
                robot_ctrl.send_raw("STOP\n")
            elif hasattr(robot_ctrl, 'send'):
                robot_ctrl.send("STOP\n")
            
            robot_ctrl.close()

        cap.release()
        cv2.destroyAllWindows()
        print("[✓] Robot Aman & Program Selesai.")


if __name__ == "__main__":
    main()
