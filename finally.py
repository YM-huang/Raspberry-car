# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
from pyzbar import pyzbar
import argparse
import datetime
import imutils
import cv2

# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# 小车按键定义
key = 8
flag = False

# 灭火电机引脚设置
OutfirePin = 2

# 超声波引脚定义
EchoPin = 0
TrigPin = 1

# RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

# 循迹红外引脚定义
TrackSensorLeftPin1 = 3  # 定义左边第一个循迹红外传感器引脚为3口
TrackSensorLeftPin2 = 5  # 定义左边第二个循迹红外传感器引脚为5口
TrackSensorRightPin1 = 4  # 定义右边第一个循迹红外传感器引脚为4口
TrackSensorRightPin2 = 18  # 定义右边第二个循迹红外传感器引脚为18口

# 舵机引脚定义
ServoPin = 23

#蜂鸣器引脚定义
buzzer = 8

# 红外避障引脚定义
AvoidSensorLeft = 12
AvoidSensorRight = 17

# 设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

# 忽略警告信息
GPIO.setwarnings(False)


# 电机引脚初始化为输出模式
# 按键引脚初始化为输入模式
# 超声波,RGB三色灯,舵机引脚初始化
# 红外避障引脚初始化
#引脚初始化函数
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(key, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
    GPIO.setup(TrackSensorRightPin1, GPIO.IN)
    GPIO.setup(TrackSensorRightPin2, GPIO.IN)
    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(buzzer, GPIO.OUT)
    GPIO.setup(OutfirePin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(AvoidSensorLeft, GPIO.IN)
    GPIO.setup(AvoidSensorRight, GPIO.IN)
    # 设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    # 设置舵机的频率和起始占空比
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)

#蜂鸣器函数
def whistle():
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.1)

# 小车前进
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车后退
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车左转
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车右转
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车原地左转
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车原地右转
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车停止
def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


# 按键检测
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
                pass


# 超声波函数
def Distance_test():
    GPIO.output(TrigPin, GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(TrigPin, GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
    t1 = time.time()
    while GPIO.input(EchoPin):
        pass
    t2 = time.time()
    print
    "distance is %d " % (((t2 - t1) * 340 / 2) * 100)
    time.sleep(0.01)
    return ((t2 - t1) * 340 / 2) * 100

#避障函数
def avoid():
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.HIGH)
    spin_left(20, 20)
    time.sleep(0.375)
    run(20, 20)
    time.sleep(0.7)
    spin_right(20, 20)
    time.sleep(0.39)
    run(20, 20)
    time.sleep(0.9)
    spin_right(20, 20)
    time.sleep(0.375)
    run(20, 20)
    time.sleep(0.8)
    spin_left(20, 20)
    time.sleep(0.375)

#避障函数2
def avoid2():
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.HIGH)
    spin_right(20, 20)
    time.sleep(0.4)
    run(20, 20)
    time.sleep(0.7)
    spin_left(20, 20)
    time.sleep(0.4)
    run(20, 20)
    time.sleep(0.8)
    spin_left(20, 20)
    time.sleep(0.4)
    run(20, 20)
    time.sleep(0.6)
    spin_right(20, 20)
    time.sleep(0.4)


# 延时2s
time.sleep(2)

# try/except语句用来检测try语句块中的错误，
# 从而让except语句捕获异常信息并处理。
try:
    init()
    key_scan()
    #检测第几次到达黑线
    count = 0
    while True:
        distance = Distance_test()
        #如果超声波距离大于20，采用循迹
        if distance > 20:
            TrackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)
            TrackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
            TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
            TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

            # 超车模块的检测
            LeftSensorValue = GPIO.input(AvoidSensorLeft);
            RightSensorValue = GPIO.input(AvoidSensorRight);

           	#当引脚为0 0 0 0时，进行判断
            if TrackSensorLeftValue1 == TrackSensorLeftValue2 == TrackSensorRightValue1 == TrackSensorRightValue2 == False:
            	#打开摄像头，启动二维码识别
                if count == 0:
                    brake()
                    # construct the argument parser and parse the arguments
                    ap = argparse.ArgumentParser()
                    ap.add_argument("-o", "--output", type=str, default="barcodes.csv",
                                    help="path to output CSV file containing barcodes")
                    args = vars(ap.parse_args())

                    # initialize the video stream and allow the camera sensor to warm up
                    print("[INFO] starting video stream...")
                    # vs = VideoStream(src=0).start()
                    cap = cv2.VideoCapture(0)

                    # open the output CSV file for writing and initialize the set of
                    # barcodes found thus far
                    csv = open(args["output"], "w")
                    found = set()
                    while True:
                        # grab the frame from the threaded video stream and resize it to
                        # have a maximum width of 400 pixels
                        ret, frame = cap.read()
                        w, h, c = frame.shape
                        r = 400 / float(w)
                        dim = (400, int(h * r))
                        frame = cv2.resize(frame, dim)
                        # find the barcodes in the frame and decode each of the barcodes
                        barcodes = pyzbar.decode(frame)

                        # loop over the detected barcodes
                        for barcode in barcodes:
                            # extract the bounding box location of the barcode and draw
                            # the bounding box surrounding the barcode on the image
                            (x, y, w, h) = barcode.rect
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                            # the barcode data is a bytes object so if we want to draw it
                            # on our output image we need to convert it to a string first
                            barcodeData = barcode.data.decode("utf-8")
                            barcodeType = barcode.type

                            # draw the barcode data and barcode type on the image
                            text = "{} ({})".format(barcodeData, barcodeType)
                            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                            # if the barcode text is currently not in our CSV file, write
                            # the timestamp + barcode to disk and update the set
                            print(barcodeData)
                            #如果二维码内容为‘car001’，令flag=true跳出循环，并亮蓝灯
                            if barcodeData == 'car001':
                                flag = True
                                GPIO.output(LED_R, GPIO.LOW)
                                GPIO.output(LED_G, GPIO.HIGH)
                                GPIO.output(LED_B, GPIO.HIGH)
                            #否则亮红灯，并响蜂鸣器
                            else:
                                GPIO.output(LED_R, GPIO.HIGH)
                                GPIO.output(LED_G, GPIO.LOW)
                                GPIO.output(LED_B, GPIO.LOW)
                                whistle()

                        # show the output frame
                        cv2.imshow("Barcode Scanner", frame)
                        key = cv2.waitKey(1) & 0xFF

                        # if the `q` key was pressed, break from the loop
                        if key == ord("q"):
                            break
                        elif (flag == True):
                            break
                    #关闭摄像头模块
                    print("[INFO] cleaning up...")
                    csv.close()
                    cap.release()
                    cv2.destroyAllWindows()
                    #小车启动风扇
                    GPIO.output(OutfirePin, not GPIO.input(OutfirePin))
                    time.sleep(1.5)
                    GPIO.output(OutfirePin, not GPIO.input(OutfirePin))
                    time.sleep(1)
                    #小车旋转180度
                    spin_right(35, 35)
                    time.sleep(0.63)
                    count += 1
                else:
                	#启动倒车入库
                    brake()
                    time.sleep(0.5)
                    back(15, 15)
                    time.sleep(0.5)
                    back(40, 0)
                    time.sleep(0.7)
                    back(15, 15)
                    time.sleep(0.5)
                    break
            # 四路循迹引脚电平状态
            # 0 0 X 0
            # 1 0 X 0
            # 0 1 X 0
            # 以上6种电平状态时小车原地右转
            # 处理右锐角和右直角的转动
            elif (TrackSensorLeftValue1 == False or TrackSensorLeftValue2 == False) and TrackSensorRightValue2 == False:
                spin_right(35, 30)
                time.sleep(0.1)

            # 四路循迹引脚电平状态
            # 0 X 0 0
            # 0 X 0 1
            # 0 X 1 0
            # 处理左锐角和左直角的转动
            elif TrackSensorLeftValue1 == False and (
                    TrackSensorRightValue1 == False or TrackSensorRightValue2 == False):
                spin_left(30, 35)
                time.sleep(0.1)

            # 0 X X X
            # 最左边检测到
            elif TrackSensorLeftValue1 == False:
                spin_left(30, 30)

            # X X X 0
            # 最右边检测到
            elif TrackSensorRightValue2 == False:
                spin_right(30, 30)

            # 四路循迹引脚电平状态
            # X 0 1 X
            # 处理左小弯
            elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
                left(0, 35)

            # 四路循迹引脚电平状态
            # X 1 0 X
            # 处理右小弯
            elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
                right(35, 0)

            elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False and (
                    LeftSensorValue == False or RightSensorValue == False):
                print(LeftSensorValue, RightSensorValue)
                run(15, 15)

            # 四路循迹引脚电平状态
            # X 0 0 X
            # 处理直线
            elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
                run(20, 20)
            # 当为1 1 1 1时小车保持上一个小车运行状态
            GPIO.output(LED_R, GPIO.LOW)
            GPIO.output(LED_G, GPIO.HIGH)
            GPIO.output(LED_B, GPIO.LOW)
        elif distance < 20:
            if (count == 0):
            	#出发时的超车函数	
                avoid()
            else:
            	#回来时的超车函数
                avoid2()
            # servo_color_carstate()
except KeyboardInterrupt:
    pass
#小车停车
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()
