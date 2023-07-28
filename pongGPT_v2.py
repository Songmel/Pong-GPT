# 패키지 임포트
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

##### 중요 환경 변수들 #####
VIDEO_SELECTION = 0  # 0번이 메인 카메라 1번부터 서브 카메라 장치들
VIDEO_WIDTH = 1000  # 화면 해상도 (1525x853)

abc = 0
CENTER_LINE = 426
LINE1_BOX = (0, 0, 1525, CENTER_LINE)
LINE2_BOX = (0, CENTER_LINE, 1525, 853)
RED_BGR = (0, 0, 255)
GREEN_BGR = (0, 255, 0)

# 초기화 변수들
LINE1_XY = None
LINE2_XY = None
LINE1_TOGGLE = False
LINE2_TOGGLE = False
ball_in_1 = False
ball_in_2 = False
line_ison = False

RALLY_COUNT = 0
FINAL_XY = None

# 주황색 탁구공 HSV 색 범위 지정
orangeLower = (18, 180, 230)
orangeUpper = (40, 255, 255)

# 파서 코딩 부분
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())
pts = deque(maxlen=args["buffer"])

# 비디오 스트리밍 시작
vs = VideoStream(src=VIDEO_SELECTION).start()
time.sleep(2.0)

# 프레임 단위 무한 루프 영역
while True:
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame
    if frame is None:
        break
    # 화면비
    frame = imutils.resize(frame, width=VIDEO_WIDTH)
    # 영상처리
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # 감지 했을 경우 (center 좌표 계산됨)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # 최소 지름 넘겼을 경우 원 그리기
        if radius > 5:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # 탁구 알고리즘 실행
            # line1
            if ball_in_1 == False and center[1] > 0 and center[1] < CENTER_LINE:
                if LINE1_TOGGLE == True:
                    LINE1_TOGGLE = False
                else:
                    LINE1_TOGGLE = True
                ball_in_1 = True
                ball_in_2 = False

                LINE1_XY = center
                print("line1 detect : ", end="")
                print(center)

            # line2
            if ball_in_2 == False and center[1] > CENTER_LINE and center[1] < 853:
                if LINE2_TOGGLE == True:
                    LINE2_TOGGLE = False
                else:
                    LINE2_TOGGLE = True
                ball_in_1 = False
                ball_in_2 = True

                LINE2_XY = center
                print("line2 detect : ", end="")
                print(center)

            # line calculating
            if LINE1_TOGGLE == True and LINE2_TOGGLE == True and line_ison == False:
                FINAL_XY = (
                    0,
                    int(
                        LINE1_XY[1]
                        - (LINE1_XY[0])
                        * (LINE1_XY[1] - LINE2_XY[1])
                        / (LINE1_XY[0] - LINE2_XY[0])
                    ),
                )
                line_ison = True
                print("FINAL_XY : ", end="")
                print(center)

            # reset for next & rally pointing
            if LINE1_TOGGLE == False and LINE2_TOGGLE == False and line_ison == True:
                RALLY_COUNT += 1
                LINE1_XY = None
                LINE2_XY = None
                FINAL_XY = None
                line_ison = False
                print("RALLY_COUNT : ", end="")
                print(RALLY_COUNT)

    else:
        ball_in_1 = False
        ball_in_2 = False

    # 트레킹 레드라인 코드
    pts.appendleft(center)
    for i in range(1, len(pts)):
        if pts[i - 1] is None or pts[i] is None:
            continue
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # 화면 표시 선 코드
    if LINE1_TOGGLE == True:
        cv2.rectangle(frame, LINE1_BOX, GREEN_BGR, 5)
    else:
        cv2.rectangle(frame, LINE1_BOX, RED_BGR, 5)

    if LINE2_TOGGLE == True:
        cv2.rectangle(frame, LINE2_BOX, GREEN_BGR, 5)
    else:
        cv2.rectangle(frame, LINE2_BOX, RED_BGR, 5)

    if line_ison == True:
        cv2.line(frame, FINAL_XY, LINE2_XY, (255, 0, 0), 5)

    cv2.line(frame, (500,0), (500,600), (0,200,0), 1)

    # show the frame to our screen
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
    elif key == ord("r"):
        LINE1_XY = None
        LINE2_XY = None
        LINE1_TOGGLE = False
        LINE2_TOGGLE = False
        ball_in_1 = False
        ball_in_2 = False
        line_ison = False
        FINAL_XY = None
        RALLY_COUNT = 0


# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()
# otherwise, release the camera
else:
    vs.release()
# close all windows
cv2.destroyAllWindows()