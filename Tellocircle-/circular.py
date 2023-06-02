import cv2
from threading import Thread
from time import sleep
from datetime import datetime
import os

# Tello SDKを使用するために必要なものをインポートする
from djitellopy import Tello

# arduinoとの通信用
import serial

# 角度計算用
import math
import numpy as np

# Telloドローンを初期化する
tello = Tello()
tello.connect()
tello.query_battery()

# ビデオを録画するフラグ
recording = False

# ストリーミングビデオからフレームを読み取る関数


def video_thread():
    video_writer = None
    global recording

    while True:
        if recording:
            if video_writer is None:
                # 録画を開始する
                video_file_name = datetime.now().strftime("%Y-%m-%d_%H.%M.%S") + ".avi"
                video_writer = cv2.VideoWriter(
                    video_file_name, cv2.VideoWriter_fourcc(*'XVID'), 30, (960, 720))

            frame = tello.get_frame_read().frame
            video_writer.write(frame)

        # キー入力を待つ
        key = cv2.waitKey(1)

        # もし「q」が押されたら、録画を停止してプログラムを終了する
        if key == ord('q'):
            recording = False
            if video_writer is not None:
                video_writer.release()
            break

        cv2.imshow("Tello", frame)

    cv2.destroyAllWindows()


def queue(src, a):
    dst = np.roll(src, -1)
    dst[-1] = a
    return dst


def console_print(data):
    ave = np.average(data)

    print('------')
    print(str(data[-1]))
    print(ave)
    print(data)


def main():

    rad = 0

    data = np.zeros(10)
    ser = serial.Serial("/dev/cu.usbmodem141201", 9600)

    # Arduinoから送られてきた値を整理する

    line = ser.readline().decode().rstrip()
    # print(line)
    data = line.split(",")
    print(len(data))
    if len(data) == 2:
        distance = data[0]
        pitch = data[1]
        print("Distance:", distance, "Pitch:", pitch)

    # 角度からの半径
    rad = -1 * math.radians(float(pitch))
    global radian
    radian = float(distance)*math.cos(rad)
    print("半径:", radian)

    # 角度からの高さ
    global height
    height = float(distance)*math.sin(rad)
    print("高さ:", height)
    print("end")


def mission():
    tello.send_rc_control(0, 0, 0, 0)
    sleep(0.1)
    # Turns motors on:
    tello.send_rc_control(-100, -100, -100, 100)
    sleep(2)
    tello.send_rc_control(0, 0, 10, 0)
    sleep(3)
    tello.send_rc_control(0, 0, 0, 0)
    sleep(2)
    rx = 0.5*radian
    ry = 0.5*radian*(-0.125)
    RX = int(rx)
    RY = int(ry)
    print(rx)
    print(ry)
    v_up = height
    # 録画フラグを有効にする
    global recording
    recording = True
    for _ in range(4):
        #
        #tello.send_rc_control(40, -5, v_up, -35)
        tello.send_rc_control(RX, RY,
                              v_up, -35)
        sleep(4)
        tello.send_rc_control(0, 0, 0, 0)
        sleep(0.5)
    tello.send_rc_control(0, 0, 0, 0)
    tello.land()
    # 録画を停止する
    recording = False

    return 0


# メインのプログラム
# if __name__ == '__main__':
main()
# ビデオストリームを開始する
tello.start_video()
# ビデオストリームからフレームを読み取るスレッドを開始する
Thread(target=video_thread).start()

# ミッションを実行する
mission()

# ビデオストリームと録画を停止する
tello.stop_video()
