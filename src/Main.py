from picamera2 import Picamera2
import cv2, numpy as np, RPi.GPIO as GPIO, time

# ===== Setup =====
FRAME_SIZE = (640, 360)
BASE_SPEED = 120
MAX_SPEED_MULT = 1.0

# Loose HSV ranges
HSV_BLUE_LOW = np.array([110, 80, 80])
HSV_BLUE_HIGH = np.array([140, 255, 200])
HSV_ORANGE_LOW = np.array([0, 50, 100])
HSV_ORANGE_HIGH = np.array([20, 255, 220])

# Minimum blue area to react
MIN_BLUE_AREA = 500  # pixels

# Motor pins
GPIO.setmode(GPIO.BCM)
ENA, IN1, IN2 = 5, 27, 22 
ENB, IN3, IN4 = 18, 23, 24
for p in [ENA, IN1, IN2, ENB, IN3, IN4]:
    GPIO.setup(p, GPIO.OUT)
pwmA, pwmB = GPIO.PWM(ENA, 1000), GPIO.PWM(ENB, 1000)
pwmA.start(0); pwmB.start(0)

def motors(l, r):
    l = max(-255, min(255, int(l * MAX_SPEED_MULT)))
    r = max(-255, min(255, int(r * MAX_SPEED_MULT)))
    GPIO.output(IN3, l>=0); GPIO.output(IN4, l<0)
    GPIO.output(IN1, r>=0); GPIO.output(IN2, r<0)
    pwmB.ChangeDutyCycle(abs(l)*100/255)
    pwmA.ChangeDutyCycle(abs(r)*100/255)
    print(f"Motors -> Left: {l}, Right: {r}")

def stop(): motors(0,0)
def forward(): motors(BASE_SPEED, BASE_SPEED)
def left(): motors(80, 200)   # gentler left for startup
def right(): motors(200, 70)

# Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size":FRAME_SIZE})
picam2.configure(config); picam2.start(); time.sleep(2)

w,h = FRAME_SIZE
y70 = int(h*0.7)

# Bottom-left box for centering
BOX_WIDTH = w//2
BOX_HEIGHT = 50
BL_BOX_X, BL_BOX_Y = 0, h - BOX_HEIGHT

# Bottom-right box for blue/orange line detection
BR_BOX_X, BR_BOX_Y = w//2, h - 70
BR_BOX_WIDTH = w//2
BR_BOX_HEIGHT = 70

# Right half black box
BR_BLACK_X, BR_BLACK_Y = w//2, h - 50
BR_BLACK_WIDTH, BR_BLACK_HEIGHT = w//2, 50
idk = True
# ===== Startup: move left until black detected in bottom-left box =====

# ===== Main loop =====
try:
    while True:
        frame = picam2.capture_array()
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Draw 70% line
        cv2.line(bgr, (0,y70),(w,y70),(0,0,255),3)

        # Bottom-left box (centering)
        cv2.rectangle(bgr, (BL_BOX_X, BL_BOX_Y), (BL_BOX_X+BOX_WIDTH, BL_BOX_Y+BOX_HEIGHT), (0,0,0), 2)
        bl_area = hsv[BL_BOX_Y:BL_BOX_Y+BOX_HEIGHT, BL_BOX_X:BL_BOX_X+BOX_WIDTH]
        bl_gray = cv2.cvtColor(cv2.cvtColor(bl_area, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
        black_percent = np.sum(bl_gray<50)/bl_gray.size*100

        # Bottom-right box (blue/orange)
        cv2.rectangle(bgr, (BR_BOX_X, BR_BOX_Y), (BR_BOX_X+BR_BOX_WIDTH, BR_BOX_Y+BR_BOX_HEIGHT), (255,0,0), 2)
        br_area = hsv[BR_BOX_Y:BR_BOX_Y+BR_BOX_HEIGHT, BR_BOX_X:BR_BOX_X+BR_BOX_WIDTH]

        mask_blue = cv2.inRange(br_area, HSV_BLUE_LOW, HSV_BLUE_HIGH)
        mask_orange = cv2.inRange(br_area, HSV_ORANGE_LOW, HSV_ORANGE_HIGH)

        # Blue bounding boxes
        cnts_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts_blue:
            x,y,wb,hb = cv2.boundingRect(c)
            cv2.rectangle(bgr, (BR_BOX_X+x, BR_BOX_Y+y), (BR_BOX_X+x+wb, BR_BOX_Y+y+hb), (255,0,0), 2)

        # Orange bounding boxes
        cnts_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts_orange:
            x,y,wo,ho = cv2.boundingRect(c)
            cv2.rectangle(bgr, (BR_BOX_X+x, BR_BOX_Y+y), (BR_BOX_X+x+wo, BR_BOX_Y+y+ho), (0,165,255), 2)

        # Right half black box
        cv2.rectangle(bgr, (BR_BLACK_X, BR_BLACK_Y), (BR_BLACK_X+BR_BLACK_WIDTH, BR_BLACK_Y+BR_BLACK_HEIGHT), (0,255,0), 2)
        br_black_area = hsv[BR_BLACK_Y:BR_BLACK_Y+BR_BLACK_HEIGHT, BR_BLACK_X:BR_BLACK_X+BR_BLACK_WIDTH]
        br_gray = cv2.cvtColor(cv2.cvtColor(br_black_area, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
        br_black_percent = np.sum(br_gray<50)/br_gray.size*100
        if idk:
            print("Starting: moving left to find black...")
            while True:
                frame = picam2.capture_array()
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

                bl_area = hsv[BL_BOX_Y:BL_BOX_Y+BOX_HEIGHT, BL_BOX_X:BL_BOX_X+BOX_WIDTH]
                bl_gray = cv2.cvtColor(cv2.cvtColor(bl_area, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
                black_percent = np.sum(bl_gray<50)/bl_gray.size*100

                if black_percent > 0:
                    print("Black detected in left -> starting normal behavior")
                    stop()
                    time.sleep(0.2)
                    idk = False
                    break
                else:
                    left()  # startup left movement
                cv2.waitKey(1)

        # Control logic
        blue_area_count = np.sum(mask_blue>0)

        if br_black_percent > 70 and black_percent == 0:
            print("Right black detected -> turning")
            motors(-150, -50)
            time.sleep(0.5)
            motors(50, 200)

        elif blue_area_count > MIN_BLUE_AREA:
            motors(200, 150)
            time.sleep(0.4)
            motors(0, 250)
            time.sleep(0.2)
            forward()
        else:
            # Centering using bottom-left box
            if black_percent == 0:
                print(1)
                motors(70, 250)
            elif black_percent < 0:
                print(2)
                left()
            elif black_percent > 5 and black_percent <= 50:
                print(3)
                right()
            elif black_percent > 50:
                print(4)
                motors(0,0)
                time.sleep(0.4)
                motors(-50, -200)
                time.sleep(0.4)
                right()
            else:
                forward()

        cv2.imshow("View", bgr)
        if cv2.waitKey(1)&0xFF==ord('q'):
            motors(0,0)
            
            break
            exit()

except KeyboardInterrupt:
    pass
finally:
    stop(); pwmA.stop(); pwmB.stop()
    GPIO.cleanup(); picam2.stop(); cv2.destroyAllWindows()
