#!/usr/bin/env python3
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import adafruit_pca9685
import board
import busio
from picamera2 import Picamera2

class ImhaSystem:
    def __init__(self, resolution=(640, 480)):
        GPIO.cleanup()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        self.PUL_PIN = 11
        self.DIR_PIN = 12
        GPIO.setup(self.PUL_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.DIR_PIN, GPIO.OUT, initial=GPIO.LOW)

        # PCA9685 ve servo ayarları
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(i2c_bus)
        self.pca.frequency = 50

        # Takip ve tetikleme eşikleri
        self.target_hold_start_time = None
        self.hold_area_threshold = 25
        self.hold_duration_threshold = 1

        # Servo açı sınırları
        self.servo_angle = 170
        self.servo_min = 125
        self.servo_max = 215

        # Step motor pozisyonu
        self.current_position = 3000
        self.max_position = 6000

        # Kamera kurulumu (Picamera2)
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": resolution}
        )
        self.picam2.configure(config)
        self.picam2.start()

        # Çerçeve boyutları ve merkez noktası
        self.frame_width, self.frame_height = resolution
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2

        # Başlangıç servo pozisyonları
        self.set_servo_angle(self.servo_angle, 0)
        self.set_servo_angle(self.servo_angle, 1)
        self.set_servo_angle(0, 3)

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
            if direction == GPIO.HIGH and self.current_position >= self.max_position:
                print("Maksimum sağ limit! Dönmeyi durduruyor.")
                break
            elif direction == GPIO.LOW and self.current_position <= 0:
                print("Maksimum sol limit! Dönmeyi durduruyor.")
                break

            GPIO.output(self.DIR_PIN, direction)
            GPIO.output(self.PUL_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.PUL_PIN, GPIO.LOW)
            time.sleep(delay)

            if direction == GPIO.HIGH:
                self.current_position += 3
            else:
                self.current_position -= 3

    def next(self):
        # Picamera2 ile çerçeve yakala
        frame = self.picam2.capture_array()
        if frame is None:
            return None, None

        # Referans noktaları çiz
        cv2.circle(frame, (self.center_x, self.center_y), self.hold_area_threshold, (0, 255, 255), 2)
        cv2.circle(frame, (self.center_x, self.center_y), 3, (255, 0, 0))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Kırmızı hedef tespiti ve hareket
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                x, y, w, h = cv2.boundingRect(largest_contour)
                object_center_x = x + w // 2
                object_center_y = y + h // 2
                cv2.circle(frame, (object_center_x, object_center_y), 3, (0, 255, 0), -1)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(frame, "Dusman", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                dx = object_center_x - self.center_x
                dy = object_center_y - self.center_y

                if abs(dx) > 20:
                    direction = GPIO.LOW if dx > 0 else GPIO.HIGH
                    self.step_motor(direction, steps=3)

                if abs(dy) > 20:
                    self.servo_angle += -1 if dy > 0 else 1
                    self.servo_angle = max(self.servo_min, min(self.servo_angle, self.servo_max))
                    self.set_servo_angle(self.servo_angle, 0)
                    self.set_servo_angle(self.servo_angle, 1)

                # Hedefi tut ve tetikle
                if (abs(dx) < self.hold_area_threshold and abs(dy) < self.hold_area_threshold):
                    if self.target_hold_start_time is None:
                        self.target_hold_start_time = time.time()
                    elif time.time() - self.target_hold_start_time >= self.hold_duration_threshold:
                        self.tetikle(80, 3)
                        time.sleep(0.5)
                        self.tetikle(0, 3)
                        self.target_hold_start_time = None
                else:
                    self.target_hold_start_time = None

        # Mavi dost tespiti
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_blue:
            if cv2.contourArea(cnt) > 600:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                cv2.putText(frame, "Dost", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", mask_red)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return None, None

        return frame, mask_red

    def cleanup(self):
        cv2.destroyAllWindows()
        GPIO.cleanup()
        self.picam2.stop()
        for i in range(16):  # Tüm kanalları temizle
            self.pca.channels[i].duty_cycle = 0
        
        # PCA9685'i tamamen kapat
        self.pca.deinit()

if __name__ == "__main__":
    system = ImhaSystem()
    try:
        while True:
            frame, mask = system.next()
            if frame is None:
                break
    finally:
        system.cleanup()
