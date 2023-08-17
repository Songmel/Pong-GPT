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
# First import the library
import pyrealsense2 as rs


print("########### Pong GPT V10 ############")

# TCP/IP 소켓통신
'''
host = "127.0.0.1"
port = 10000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((host, port))
server.listen()
print("Connection Ready...")

# clients
client_actu = None
client_arm = None

# 두 명의 클라이언트를 받음
for i in range(2):
    if i == 1:
        print("Next Connection Waiting...")
    client, address = server.accept()
    print("Connected with {}".format(str(address)))
    check_token = client.recv(1).decode()
    if check_token == "a":
        print("Actuator Connected!")
        client_actu = client
    elif check_token == "r":
        print("Robot Arm Connected!")
        client_arm = client
    else:
        print("Connection Error")
        exit()


# 엑추에이터 값 전달
def actu_send(client_actu, fin_move, fin_eta):
    try:
        message = "{0},{1}".format(fin_move, fin_eta)
        client_actu.send(message.encode(encoding="utf-8"))

    except:
        client_actu.close()


# 로봇팔 값 전달
def arm_send(client_arm, fin_eta, fin_angle):
    try:
        message = "{0},{1}".format(fin_eta, fin_angle)
        client_arm.send(message.encode(encoding="utf-8"))
    except:
        client_arm.close()

'''
##### 중요 환경 변수들 #####
VIDEO_SELECTION = 2  # 0번부터 카메라 포트 찾아서 1씩 올려보기
VIDEO_WIDTH = 1000  # 화면 가로 넓이
WIDTH_CUT = 160
CENTER_LINE = 340  # 세로 센터 라인
NET_LINE = 250  # 네트 라인

CATCH_FRAME = 3 # 좌표 계산 프레임 수
MIN_GAP = 20 # 최소 감지 속도 (px)
MOVE_FIX = 0 # 최종 좌표 미세 조정
HIT_LINE = 750 - 100 # 타격 명령 감지 범위 (px)
HIT_DELAY = 10 # 감지 이후 타격까지 딜레이 (ms)
LINE_DEACTIVE_TIME = 0.8 # 감지 비활성화 시간 (sec)
ACTU_WAIT_TIME = 300 # 엑추에이터 이동 후 대기 시간 (ms)

DEPTH = 1.3



# 초기화 변수들
line_on = False
hit_on = False
FINAL_MOVE = 0  # 단위 cm

# 주황색 탁구공 HSV 색 범위 지정 (창문쪽 형광등 두 개 키고 문쪽 형광등 한 개 껐을때 기준)
orangeLower = (1, 130, 240)
orangeUpper = (30, 255, 255)

# Create a pipeline
pipeline = rs.pipeline()
# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
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


# 파서 코딩 부분
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

# 데큐 생성
pts = deque(maxlen=args["buffer"])
line_xy = deque(maxlen=2)  # 단위 px
temp_move = deque()  # 단위 px


# Line Activater 쓰레드 함수
def line_activator(ETA):
    line_xy.clear()
    temp_move.clear()

    global line_on
    line_on = True
    print("Line Activated / Detecting LOCK")
    time.sleep(ETA)
    global hit_on
    hit_on = False
    line_on = False
    print("Line Deactivated / Detecting UNLOCK")

# 비디오 스트리밍 시작
#vs = VideoStream(src=VIDEO_SELECTION).start()
#time.sleep(2.0)

# Start streaming
profile = pipeline.start(config)
# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)
# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale
# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# 프레임 단위 무한 루프 영역
while True:
    frames = pipeline.wait_for_frames()
    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        continue

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    grey_color = 153
    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    # Render images:
    #   depth align to color on left
    #   depth on right
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    images = np.hstack((bg_removed, depth_colormap))

    cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
    cv2.imshow('Align Example', images)

    # 화면비 (680x750)
    frame = imutils.resize(color_image, width=VIDEO_WIDTH)
    frame = frame[0:750, WIDTH_CUT : 1000 - WIDTH_CUT]
    # 영상처리
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cv2.imshow("mask", mask)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    realcenter = None

    # 감지 했을 경우 (center 좌표 계산됨)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if 0 <= x <= 640 and 0 <= y <= 480:
            dist = aligned_depth_frame.get_distance(int(x), int(y))
            dist = round(dist, 4)
            dist *= 100
            print(dist)

        # 뎁스 알고리즘
        realcenter = [0, 0]
        # x좌표 수정
        if center[1] < 500:
            if center[0] < 340:
                realcenter[0] = int(
                    center[0] + ((340 - center[0]) / 4) * ((500 - center[1]) / 500) * DEPTH
                )
            elif center[0] > 340:
                realcenter[0] = int(
                    center[0] - ((center[0] - 340) / 4) * ((500 - center[1]) / 500) * DEPTH
                )
            else:
                realcenter[0] = center[0]
        else:
            realcenter[0] = center[0]

        # y좌표 수정
        realcenter[1] = center[1] 


        # 탁구 알고리즘
        if line_on == False:
            line_xy.append(realcenter)
            if len(line_xy) == 2:
                if line_xy[0][1] + MIN_GAP < line_xy[1][1]:
                    temp_move.append(
                        int(
                            (750 - line_xy[0][1])
                            * (line_xy[0][0] - line_xy[1][0])
                            / (line_xy[0][1] - line_xy[1][1])
                            + line_xy[0][0]
                        )
                    )

                if line_xy[0][1] > line_xy[1][1]:
                    line_xy.clear()


        if len(temp_move) == CATCH_FRAME:
            temp_move.popleft()

            # move 계산
            temp_move_sum = 0
            for i in range(CATCH_FRAME - 1):
                temp_move_sum += temp_move.popleft()
            FINAL_MOVE = int(temp_move_sum / (CATCH_FRAME - 1) * (152.5 / 680))
            if FINAL_MOVE < 76:
                FINAL_MOVE += int((76 - FINAL_MOVE) * MOVE_FIX)
            else:
                FINAL_MOVE -= int((FINAL_MOVE - 76) * MOVE_FIX)

            print(
                "FINAL MOVE : {0}cm".format(
                    FINAL_MOVE
                )
            )


            # 감지 대기 쓰레드
            lineact_tr = threading.Thread(
                target=line_activator, args=(LINE_DEACTIVE_TIME,), daemon=True
            )
            lineact_tr.start()

            # 엑추에이터 송신 쓰레드
            '''
            actu_tr = threading.Thread(
                target=actu_send,
                args=(
                    client_actu,
                    FINAL_MOVE,
                    ACTU_WAIT_TIME,
                ),
            )
            actu_tr.start()
            '''

        
        if realcenter[1] > HIT_LINE and line_on == True and hit_on == False:
            if center[1] > 700:
                HIT_DELAY = 1
            else:
                HIT_DELAY = 10

            # 로봇팔 송신 쓰레드
            '''
            arm_tr = threading.Thread(
                target=arm_send,
                args=(client_arm, HIT_DELAY, 100),
            )
            arm_tr.start()
            '''
            hit_on = True
            print("Hit! : {0}".format(realcenter))
            

    # rgb 트레킹 레드라인 코드
    pts.appendleft(realcenter)
    for i in range(1, len(pts)):
        if pts[i - 1] is None or pts[i] is None:
            continue
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # 화면 표시 선 코드
    # 중앙선
    cv2.line(frame, (CENTER_LINE, 0), (CENTER_LINE, 750), (255, 255, 255), 2)

    # 네트선
    cv2.line(frame, (0, NET_LINE), (VIDEO_WIDTH, NET_LINE), (255, 255, 255), 2)

    # 화면 띄우기
    cv2.imshow("Frame", frame)

    # q : 종료 r : 리셋
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("r"):
        line_xy.clear()
        temp_move.clear()
        line_on = False
        hit_on = False
        FINAL_MOVE = None

if not args.get("video", False):
    vs.stop()
else:
    vs.release()

cv2.destroyAllWindows()
