import time
import serial
import serial.tools.list_ports


class SManage:
    """Class untuk mengelola komunikasi Serial dari Python ke ESP32 Pengendali Servo.

    Mendukung auto-reconnect, pengiriman sudut steering, dan perintah STOP.
    """

    def __init__(self, port: str = None, baudrate: int = 9600, timeout: float = 1.0, min_interval: float = 0.03):
        """Inisialisasi Serial Manager.

        :param port: Nama port (misal 'COM3' di Windows atau '/dev/ttyUSB0' di Linux).
                     Jika None, class akan mencoba auto-detect port ESP32.
        :param baudrate: Baudrate komunikasi Serial (harus sama dengan Serial.begin di ESP32, yaitu 9600).
        :param timeout: Serial read timeout (detik).
        :param min_interval: Waktu jeda minimum antar pengiriman data (detik) untuk mencegah buffer overflow.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.min_interval = min_interval

        self.ser = None
        self.last_send_time = 0.0
        self.last_sent_angle = None

    def auto_detect_port(self) -> str:
        """Mencari port USB Serial yang aktif secara otomatis."""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Seringkali ESP32 terdeteksi sebagai CP210x, CH340, atau FTDI
            if any(vid in (p.description or "") for vid in ["CP210", "CH340", "UART", "USB Serial"]):
                return p.device
        # Jika tidak ketemu keyword spesifik, ambil port pertama jika ada
        if ports:
            return ports[0].device
        return None

    def connect(self) -> bool:
        """Buka koneksi serial ke ESP32."""
        if self.ser and self.ser.is_open:
            return True

        if not self.port:
            detected = self.auto_detect_port()
            if detected:
                self.port = detected
            else:
                print("[SManage] Error: Port Serial tidak ditemukan!")
                return False

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2.0)  # Beri waktu ESP32 untuk auto-reset setelah koneksi Serial dibuka
            
            # Buang data awal dari ESP32 (seperti "ESP32 Ready...")
            if self.ser.in_waiting:
                self.ser.read_all()

            print(f"[SManage] Terhubung ke ESP32 pada port: {self.port} ({self.baudrate} bps)")
            return True
        except serial.SerialException as e:
            print(f"[SManage] Gagal terhubung ke port {self.port}: {e}")
            self.ser = None
            return False

    def send_raw(self, message: str) -> bool:
        """Kirim string mentah dengan karakter newline '\\n' di akhirnya."""
        if not self.ser or not self.ser.is_open:
            # Coba reconnect jika terputus
            if not self.connect():
                return False

        current_time = time.time()
        # Rate Limiting: Mencegah spam data berlebihan yang bisa bikin lag ESP32
        if (current_time - self.last_send_time) < self.min_interval:
            return False

        try:
            formatted_msg = f"{message.strip()}\n"
            self.ser.write(formatted_msg.encode('utf-8'))
            self.last_send_time = current_time
            return True
        except serial.SerialException as e:
            print(f"[SManage] Error saat mengirim data: {e}")
            self.close()
            return False

    def send_angle(self, angle: int, force: bool = False) -> bool:
        """Kirim target sudut steering ke ESP32.

        :param angle: Nilai derajat (misal 45 - 135).
        :param force: Jika True, tetap kirim meskipun nilainya sama dengan nilai sebelumnya.
        """
        angle = int(angle)
        
        # Mencegah pengiriman berulang untuk angka sudut yang persis sama
        if not force and angle == self.last_sent_angle:
            return True

        if self.send_raw(str(angle)):
            self.last_sent_angle = angle
            return True
        return False

    def send_stop(self) -> bool:
        """Kirim perintah 'STOP' untuk mengembalikan steering ke sudut netral (90)."""
        if self.send_raw("STOP"):
            self.last_sent_angle = 90
            return True
        return False

    def is_connected(self) -> bool:
        """Cek apakah koneksi serial aktif."""
        return self.ser is not None and self.ser.is_open

    def close(self):
        """Tutup koneksi serial secara aman."""
        if self.ser and self.ser.is_open:
            try:
                self.send_stop()  # Kembalikan ke posisi netral sebelum putus
                time.sleep(0.1)
                self.ser.close()
                print("[SManage] Koneksi Serial ditutup.")
            except Exception as e:
                print(f"[SManage] Error saat menutup koneksi: {e}")
            finally:
                self.ser = None