import sys
import rospy
import time
import cv2
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#------------------------------------------------------------------------
# Dronun bağlantı adresi        
connection_string = "127.0.0.1:14550"
# Ros kamera adresi
webcam = "/webcam/image_raw"
# Dronun hızı
hız = 1 
# Dronun bağlanması 
iha=connect(connection_string,wait_ready=True,timeout=100)
# Önceki uçuşlardan kalan verilerin temizlenmesi
iha.flush()
print("Drona bağlandı")
time.sleep(2)
#------------------------------------------------------------------------

# Dronu hareket ettirme fonksiyonu
def set_iha_body(vx, vy, vz):

  msg = iha.message_factory.set_position_target_local_ned_encode( 
      0,
      0, 0,
      mavutil.mavlink.MAV_FRAME_BODY_NED,
      0b0000111111000111,
      0, 0, 0, 
      vx, vy, vz,
      0, 0, 0, 
      0, 0)
  iha.send_mavlink(msg)
  iha.flush()

def find_green_square(image):
    # Giriş görüntüsünü BGR formatından HSV formatına dönüştürüyorum
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h,w,d = image.shape
    a = int(h/2)
    b = int(w/2)
    # Yeşil rengin HSV değer aralığını belirliyorum
    lower_green = np.array([40, 50, 50])  # Düşük HSV değerleri
    upper_green = np.array([80, 255, 255])  # Yüksek HSV değerleri

    # HSV görüntüsünde yeşil renk aralığını maskeleme
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Maskeyi uygulayarak yeşil karenin konturlarını buluyorum
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # En büyük konturu buluyorum
        largest_contour = max(contours, key=cv2.contourArea)

        # Konturun çevreleyen dörtgeni buluyorum
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Kare çevresinin orta noktasını hesaplayın
        center_x = x + (w // 2)
        center_y = y + (h // 2)
        # Kareyi çizdirin
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # Karenin orta noktasını çiziyor
        cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), cv2.FILLED)
        # Dron kamerasının orta noktasını çiziyor
        cv2.circle(image, (a, b), 5, (255, 255, 0), cv2.FILLED)

        # yeşil karenin Orta nokta koordinatlarını döndür
        return center_x, center_y, a, b

    else:
        # Yeşil kare bulunamadığında None döndür
        return None

def drone_indir():
    # Modu LAND olarak ayarla
    iha.mode = VehicleMode("LAND")
    print("Drone iniyor'")
    # İniş durumunu kontrol etme 
    while True:
        altitude = iha.location.global_relative_frame.alt
        print(f"Su anki yukseklik{iha.location.global_relative_frame.alt}")
        time.sleep(0.5)
        if iha.mode == VehicleMode('LAND') and altitude < 1:
            iha.close()
            time.sleep(2)
            sys.exit()
        elif iha.mode != VehicleMode('LAND'):
            print("İniş yapılamıyor!!!")

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()

        # Ros görüntüleri alınır ve cllbeck foksiyonu çalıştırılır 
        self.image_sub = rospy.Subscriber(webcam,Image,self.callback)

    def callback(self,data):
        try:
            # ROS görüntülerini OpenCV formatına dönüştür 
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        except CvBridgeError as e:
            print(e)
      
        # Yeşil karenin kordinatlarını alırız
        center = find_green_square(cv_image)
        w,h,d = cv_image.shape
        # Bir şey tespit edilmediyse dorn ilerlemeye devam eder 
        if center is None:
            set_iha_body(hız, 0, 0)

        # Dronun merkezini yeşil karenin merkezine yaklaştırı
        if center is not None:
            if abs(center[2] - center[0]) <= 50 and abs(center[3] - center[1]) <= 50:
                drone_indir()
            else:
                if center[2] - center[0] > 50:
                    print("asddsa")
                    set_iha_body(0, hız, 0)
                elif center[2] - center[0] < -50:
                    set_iha_body(0, -hız, 0)
                if center[3] - center[1] > 50:
                    set_iha_body(hız, 0, 0)
                elif center[3] - center[1] < -50:
                    set_iha_body(-hız, 0, 0)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def arm_ol_ve_yuksel(hedef_yukseklik):
    if not hasattr(arm_ol_ve_yuksel, "has_run"):
        while iha.is_armable==False:
            print("Arm ici gerekli sartlar saglanamadi.")
            time.sleep(1)
        print("Iha su anda armedilebilir")
        
        iha.mode=VehicleMode("GUIDED")
        while iha.mode=='GUIDED':
            print('Guided moduna gecis yapiliyor')
            time.sleep(1.5)

        print("Guided moduna gecis yapildi")
        iha.armed=True
        while iha.armed is False:
            print("Arm icin bekleniliyor")
            time.sleep(1)

        print("Ihamiz arm olmustur")
        
        iha.simple_takeoff(hedef_yukseklik)
        while iha.location.global_relative_frame.alt<=hedef_yukseklik*0.94:
            print(f"Su anki yukseklik{iha.location.global_relative_frame.alt}")
            time.sleep(0.5)
        print("Takeoff gerceklesti")
        arm_ol_ve_yuksel.has_run = True

def main(args):
    arm_ol_ve_yuksel(10)
    time.sleep(2)
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)