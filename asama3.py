#!/usr/bin/env python3
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import adafruit_pca9685
import board
import busio
from pyzbar.pyzbar import decode
from picamera2 import Picamera2

class Qr2System:
    def __init__(self, resolution=(640, 480)):
        GPIO.cleanup()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        self.PUL_PIN = 11
        self.DIR_PIN = 12
        GPIO.setup(self.PUL_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.DIR_PIN, GPIO.OUT, initial=GPIO.LOW)

        # PCA9685 Ayarları
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(i2c_bus)
        self.pca.frequency = 50

        # Servo ayarları
        self.servo_min = 125
        self.servo_max = 215
        self.current_servo_angle = 0

        # Motor ve Görüntü Ayarları
        self.image_width, self.image_height = resolution
        self.current_step_angle = 3000
        self.max_position = 6000

        # İmha Kontrol Değişkenleri
        self.target_hold_start_time = None
        self.hold_area_threshold = 25
        self.hold_duration_threshold = 2

        # Referans ve takip durumları
        self.referans_sekil = None
        self.letter_rotation_done = False
        self.tracking_active = False
        self.detected_text = ""
        self.qr_detected = False
        self.last_detected_shape = None
        self.shape_detected = False

        # Renk Ayarları
        self.renk_araliklari = {
            'kirmizi': {'dusuk1': np.array([0, 120, 70]),  'yuksek1': np.array([10, 255, 255]),
                        'dusuk2': np.array([170, 120, 70]), 'yuksek2': np.array([180, 255, 255])},
            'mavi':    {'dusuk': np.array([100, 150, 50]),  'yuksek': np.array([140, 255, 255])},
            'yesil':   {'dusuk': np.array([35,100,50]),   'yuksek': np.array([85,255,255])}
        }
        self.kernel = np.ones((5,5), np.uint8)

        # Kamera kurulumu (Picamera2)
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (self.image_width, self.image_height)}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def set_servo_angle(self, angle, servo_channel):
        if servo_channel == 0 :
            angle = max(self.servo_min, min(angle, self.servo_max))
            gercek_servo_aci = 330 - angle
            pulse = int(150 + (gercek_servo_aci / 330) * 450)
        
        elif servo_channel==3:
            angle = max(0, min(angle, self.servo_max))
            gercek_servo_aci = 330 - angle
            pulse = int(150 + (gercek_servo_aci / 330) * 450)
        else:
            angle = max(0, min(angle, 180))
            pulse = int(150 + (angle / 180) * 450)
        self.pca.channels[servo_channel].duty_cycle = pulse * 65535 // 4096
        
    
    def tetikle(self, angle, ch=3):
     
        self.set_servo_angle(angle, ch)
      
        #inv_angle = 180 - angle
        self.set_servo_angle(angle, ch)

    def step_motor(self, direction, steps, delay=0.001):
        for _ in range(steps):
            if direction == GPIO.HIGH and self.current_step_angle >= self.max_position:
                print("Maksimum sağ limit!")
                break
            if direction == GPIO.LOW and self.current_step_angle <= 0:
                print("Maksimum sol limit!")
                break
            GPIO.output(self.DIR_PIN, direction)
            GPIO.output(self.PUL_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.PUL_PIN, GPIO.LOW)
            time.sleep(delay)
            self.current_step_angle += 3 if direction == GPIO.HIGH else -3

    def adjust_servo_angle(self, direction):
        k = 0.2
        delta = int(k * 10)
        self.current_servo_angle += delta if direction == "up" else -delta
        self.current_servo_angle = max(self.servo_min, min(self.current_servo_angle, self.servo_max))
        self.set_servo_angle(self.current_servo_angle, 0)
        self.set_servo_angle(self.current_servo_angle, 1)
    
    def tetikle(self, angle, ch=3):

        self.set_servo_angle(angle, ch)
      
        #inv_angle = 250-angle
        self.set_servo_angle(angle, ch)

    def rotate_by_letter(self, letter):
        direction = GPIO.LOW if letter.upper() == 'A' else GPIO.HIGH
        if letter.upper() == 'A':
            print("A QR kodu tespit edildi, sola dönülüyor...")
            self.step_motor(direction, steps=400)
            self.current_step_angle -= 900
        elif letter.upper() == 'B':
            print("B QR kodu tespit edildi, sağa dönülüyor...")
            self.step_motor(direction, steps=400)
            self.current_step_angle += 900

    def renk_tespit(self, hsv, renk):
        if renk == 'kirmizi':
            m1 = cv2.inRange(hsv, self.renk_araliklari[renk]['dusuk1'], self.renk_araliklari[renk]['yuksek1'])
            m2 = cv2.inRange(hsv, self.renk_araliklari[renk]['dusuk2'], self.renk_araliklari[renk]['yuksek2'])
            mask = cv2.bitwise_or(m1, m2)
        else:
            mask = cv2.inRange(hsv, self.renk_araliklari[renk]['dusuk'], self.renk_araliklari[renk]['yuksek'])
        mask = cv2.erode(mask, self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=1)
        return mask

    def en_buyuk_kontur_bul(self, mask):
        c, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i in c:
            if cv2.contourArea(i) < 50:
                return None
        return c

    def sekil_tespit(self, approx):
        n = len(approx)
        if n == 3:
            return "Ucgen"
        if n == 4:
            return "Dortgen"
        return "Daire"

    def sekil_ozellikleri_bul(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for renk in ['yesil','kirmizi','mavi']:
            mask = self.renk_tespit(hsv, renk)
            c = self.en_buyuk_kontur_bul(mask)
            if c is not None:
                for i in c:
                    eps = 0.04 * cv2.arcLength(i, True)
                    approx = cv2.approxPolyDP(i, eps, True)
                    return {'renk':renk, 'sekil':self.sekil_tespit(approx), 'kontur':i, 'approx':approx}
        return None

    def belirli_renk_sekil_bul(self, frame, hedef_renk, hedef_sekil):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self.renk_tespit(hsv, hedef_renk)
        c = self.en_buyuk_kontur_bul(mask)
        if c is not None:
            for i in c:
                eps = 0.04 * cv2.arcLength(i, True)
                approx = cv2.approxPolyDP(i, eps, True)
                if self.sekil_tespit(approx) ==  hedef_sekil:
                    return {'renk':hedef_renk, 'sekil':self.sekil_tespit(approx), 'kontur':i, 'approx':approx}
        return None

    def next(self):
        frame = self.picam2.capture_array()
        if frame is None:
            return None, None
        #print(f"{self.current_step_angle}")
        # QR kodu yalnızca henüz dönülmemişse tara
        if not self.letter_rotation_done:
            decoded_objects = decode(frame)
            for obj in decoded_objects:
                qr_data = obj.data.decode('utf-8')
                if qr_data in ["A", "B"]:
                    self.detected_text = qr_data
                    self.qr_detected = True
                    points = obj.polygon
                    if len(points) == 4:
                        pts = np.array([(p.x, p.y) for p in points], dtype=np.int32)
                        cx = pts[:,0].mean().astype(int)
                        cy = pts[:,1].mean().astype(int)
                        cv2.polylines(frame, [pts], True, (0,255,0), 1)
                        cv2.putText(frame, f"{qr_data} Algilandi!", (cx-60, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        # Şekil takibi ve imha
        if self.referans_sekil and self.tracking_active:
            hedef = self.belirli_renk_sekil_bul(frame, self.referans_sekil['renk'],self.referans_sekil['sekil'])
            if hedef and hedef['sekil'] == self.referans_sekil['sekil']:
                self.shape_detected = True
                self.last_detected_shape = hedef
                cv2.drawContours(frame, [hedef['approx']], 0, (0,255,0), 3)
                M = cv2.moments(hedef['kontur'])
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    dx = cx - (self.image_width//2)
                    dy = cy - (self.image_height//2)

                    if abs(dx) > 30:
                        direction = GPIO.LOW if dx > 0 else GPIO.HIGH
                        self.step_motor(direction, steps=5)

                    if abs(dy) > 10:
                        self.adjust_servo_angle("down" if dy > 0 else "up")
                        

                    if abs(dx) < self.hold_area_threshold and abs(dy) < self.hold_area_threshold:
                        if self.target_hold_start_time is None:
                            self.target_hold_start_time = time.time()
                        elif time.time() - self.target_hold_start_time >= self.hold_duration_threshold and abs(self.current_step_angle-3000)>300:
                            print(" İmha gerçekleştiriliyor!")
                            self.tetikle(80, 3)
                            time.sleep(0.5)
                            self.tetikle(0, 3)
                            time.sleep(0.5)
                            self.target_hold_start_time = None
                    else:
                        self.target_hold_start_time = None
            else:
                self.shape_detected = False
        else:
            shap = self.sekil_ozellikleri_bul(frame)
            if shap:
                self.shape_detected = True
                self.last_detected_shape = shap
                cv2.drawContours(frame, [shap['approx']], 0, (0,255,0), 3)
                
                
        cv2.circle(frame, ((self.image_width//2), (self.image_height//2)), self.hold_area_threshold, (0, 255, 255), 2)
        cv2.circle(frame, ((self.image_width//2), (self.image_height//2)), 1, (255, 0, 0))
        
        
        status = "Durum: "
        if not self.referans_sekil:
            status += "Referans şekil bekleniyor"
        elif not self.qr_detected:
            status += "QR Kod bekleniyor"
        elif not self.letter_rotation_done:
            status += "Angajman emri bekleniyor"
        else:
            status += "Takip ve imha aktif"
        cv2.putText(frame, status, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        cv2.imshow("QR Kod ve Sekil Tespiti", frame)

        key = cv2.waitKey(5) & 0xFF
        if key == ord('q'):
            return None, None
        elif key == ord('s') and self.shape_detected and not self.referans_sekil:
            self.referans_sekil = self.last_detected_shape
            print(f"✔ Referans alındı: {self.referans_sekil['renk']} {self.referans_sekil['sekil']}")
        elif key == 13 and self.referans_sekil and self.qr_detected and not self.letter_rotation_done:
            self.rotate_by_letter(self.detected_text)
            self.letter_rotation_done = True
            self.tracking_active = True
            print(f"{self.detected_text} QR koduna göre dönüldü, takip başladı.")
        elif key == ord('r'):
            self.referans_sekil = None
            self.letter_rotation_done = False
            self.tracking_active = False
            self.qr_detected = False
            self.detected_text = ""
            print("Sistem sıfırlandı.")

        return frame, None

    def cleanup(self):
        cv2.destroyAllWindows()
        GPIO.cleanup()
        self.set_servo_angle(90, 0)
        self.picam2.stop()

if __name__ == "__main__":
    system = Qr2System()
    try:
        while True:
            frame, _ = system.next()
            if frame is None:
                break
    finally:
        system.cleanup()
