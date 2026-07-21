# FIRA OpenCV Line Tracking

Proyek ini adalah sistem pelacak garis (*line tracking*) berbasis **OpenCV** dan **Python** yang dirancang untuk robot autonomous pada kompetisi **FIRA (Federation of International Robot-sports Association)**.

---

## 📝 Deskripsi Proyek

Sistem ini berfungsi untuk memproses *feed* kamera secara *real-time*, mendeteksi posisi garis menggunakan metode segmentasi warna (HSV/Thresholding), dan menghitung estimasi arah pergerakan robot. Output dari sistem ini berupa sinyal kontrol (seperti kontrol PID) yang dapat dikirimkan ke mikrokontroler (ESP32, Arduino, atau ROS) untuk mengendalikan arah dan kecepatan motor robot.

### Fitur Utama:
- **Deteksi Garis Real-time**: Pemrosesan citra cepat menggunakan OpenCV.
- **Filtering Warna Adaptive**: Pengaturan threshold HSV untuk berbagai kondisi pencahayaan.
- **Integrasi Hardware**: Komunikasi data via Serial (UART) atau UDP ke mikrokontroler.

---

## 👥 Authors

1. zaenal abidin
2. ahmad khoirudin
3. Nabillah Ilham
4. rizky ramadhani
5. A. VIKY ADI S
6. M. Syiham Lazuardi
7. Ali Zaenal Abidin
