# 패키지 임포트
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import threading
import socket
import pyrealsense2 as rs

print("########### Pong GPT V5 ############")

# Realsense Booting up
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

##### 중요 환경 변수들 #####
VIDEO_SELECTION = 1  # 0번부터 카메라 포트 찾아서 1씩 올려보기
VIDEO_WIDTH = 1000  # 화면 가로 넓이
WIDTH_CUT = 160
CENTER_LINE = 340  # 세로 센터 라인
NET_LINE = 640  # 네트 라인

CATCH_FRAME = 3
MIN_GAP = 10
ETA_FIX = 80
CENTER_BOUND = 150

# 초기화 변수들
line_on = False

FINAL_MOVE = 0  # 단위 cm
FINAL_ETA = 0  # 단위 ms
FINAL_ANGLE = 0  # 단위 tangent

# 주황색 탁구공 HSV 색 범위 지정 (창문쪽 형광등 두 개 키고 문쪽 형광등 한 개 껐을때 기준)
orangeLower = (1, 130, 240)
orangeUpper = (30, 255, 255)

# 파서 코딩 부분
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

# 데큐 생성
pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=0).start()
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])
# allow the camera or video file to warm up
time.sleep(2.0)

line_xy = deque(maxlen=2)  # 단위 px
time_xy = deque(maxlen=2)  # 단위 s
temp_move = deque()  # 단위 px
temp_speed = deque()  # 단위 px/ms


# Line Activater 쓰레드 함수
def line_activator(ETA):
    global line_on
    line_on = True
    print("Line Activated / Detecting LOCK")
    time.sleep(ETA)
    line_on = False
    print("Line Deactivated / Detecting UNLOCK")
    line_xy.clear()
    time_xy.clear()
    temp_move.clear()
    temp_speed.clear()

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1.7 #1.7 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# 프레임 단위 무한 루프 영역
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        #
        #
        #
        #
        #
        #
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        #depth_frame = frames.get_depth_frame()
        #color_frame = frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue
        #Depth image, color image
        #
        #
        #
        #
        #
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)

        #From version 6, ball tracking
        # 화면비 맞추기 (680x750)
        color_image = imutils.resize(color_image, width=VIDEO_WIDTH)
        depth_image = imutils.resize(depth_image, width=VIDEO_WIDTH)
        color_image = color_image[0:750, WIDTH_CUT: 1000 - WIDTH_CUT]
        depth_image = depth_image[0:750, WIDTH_CUT: 1000 - WIDTH_CUT]
        # 영상처리
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        blurred = cv2.GaussianBlur(color_image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, orangeLower, orangeUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow("mask", mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # 감지 했을 경우 (center 좌표 계산됨)
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # rgb 트레킹 레드라인 코드
            pts.appendleft(center)
            for i in range(1, len(pts)):
                if pts[i - 1] is None or pts[i] is None:
                    continue
                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(color_image, pts[i - 1], pts[i], (0, 0, 255), thickness)

            #오류인지 확인해야 함
            #
            #
            #
            dist = aligned_depth_frame.get_distance(center[0], center[1])
            dist *= 100

            print(dist)
            #realcenter = (center[0] * dist / 135, center[1] * dist / 135)

            # 탁구 알고리즘
            if line_on == False:
                line_xy.append(center)
                time_xy.append(time.time())
                if len(line_xy) == 2:
                    if line_xy[0][1] + MIN_GAP < line_xy[1][1]:
                        temp_move.append(
                            int(
                                (1220 - line_xy[0][1])
                                * (line_xy[0][0] - line_xy[1][0])
                                / (line_xy[0][1] - line_xy[1][1])
                                + line_xy[0][0]
                            )
                        )
                        temp_speed.append(
                            int(
                                (line_xy[0][1] - line_xy[1][1])
                                / ((time_xy[1] - time_xy[0]) * 1000)
                            )
                        )

            if len(temp_move) == CATCH_FRAME:
                temp_move.popleft()
                temp_speed.popleft()

                temp_move_sum = 0
                for i in range(CATCH_FRAME - 1):
                    temp_move_sum += temp_move.popleft()
                FINAL_MOVE = int(temp_move_sum / (CATCH_FRAME - 1) * (152.5 / 680))

                temp_speed_sum = 0
                for i in range(CATCH_FRAME - 1):
                    temp_speed_sum += temp_speed.popleft()
                FINAL_ETA = (
                        int((1220 - line_xy[1][1]) / (temp_speed_sum / (CATCH_FRAME - 1)))
                        + ETA_FIX
                )

                FINAL_ANGLE = (1220 - line_xy[1][1]) / (
                        line_xy[1][0] - FINAL_MOVE * (680 / 152.5)
                )

                print(
                    "FINAL MOVE : {0}cm / FINAL ETA : {1}ms / FINAL ANGLE : {2}".format(
                        FINAL_MOVE, FINAL_ETA, FINAL_ANGLE
                    )
                )

                # 감지 대기 쓰레드
                lineact_tr = threading.Thread(
                    target=line_activator, args=(FINAL_ETA / 1000,), daemon=True
                )
                lineact_tr.start()

        # 화면 표시 선 코드
        # 중앙선
        cv2.line(color_image, (CENTER_LINE, 0), (CENTER_LINE, NET_LINE), (255, 255, 255), 2)

        # 네트선
        cv2.line(color_image, (0, NET_LINE), (VIDEO_WIDTH, NET_LINE), (255, 255, 255), 2)

        images = np.hstack((color_image, depth_colormap))
        images = imutils.resize(images, width=500)

        # 화면 띄우기
        cv2.imshow("Pong GPT V5", images)
        cv2.imshow("mask", mask)

        # q : 종료 r : 리셋
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyAllWindows()
            break
        elif key == ord("r"):
            line_xy.clear()
            time_xy.clear()
            temp_move.clear()
            temp_speed.clear()
            line_on = False
            FINAL_MOVE = None
            FINAL_ETA = None
            FINAL_ANGLE = None

finally:
    pipeline.stop()

