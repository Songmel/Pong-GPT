# import the necessary packages

# 콜렉션 모듈에서 deque라는 데이터 처리 함수
from collections import deque

# imutils 비디오스트림 모듈 임포트
from imutils.video import VideoStream

# numpy 모듈 임포트
import numpy as np

# argparse 모듈 임포트 (파이썬 스크립트를 명령 프롬프트에서 실행할 때 명령행 인자를 파싱할 수 있게 한다)
import argparse

# OpenCV 모듈 임포트
import cv2

# imutils 모듈 임포트 (이미지 프로세싱 관련된 유용한 기능들 있다)
import imutils

# time 모듈 임포트
import time

##### CHANGABLE VARIABLES #####
CHECK_RANGE = 200
LINE1_XY = None
LINE2_XY = None
LINE1_TOGGLE = False
LINE2_TOGGLE = False
ball_isout = True
line_ison = False
FINAL_XY = None


# construct the argument parse and parse the arguments
# parser를 만든다 이름은 ap
ap = argparse.ArgumentParser()
# add_argument 메서드로 받아들일 인수를 추가해나간다
# -v 로 비디오 파일을 선택해서 재생할 수 있다.
ap.add_argument("-v", "--video", help="path to the (optional) video file")
# -b 로 최대 버퍼 사이즈를 지정할 수 있다 기본은 64
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "Orange"
# ball in the HSV color space, then initialize the
# list of tracked points
# @@@오랜지 Hsv 색 범위 지정@@@
orangeLower = (6, 170, 100)
orangeUpper = (24, 255, 255)
pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    # @@@비디오 장치 지정@@@
    vs = VideoStream(src=1).start()
# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])
# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:
    # grab the current frame
    frame = vs.read()
    # handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args.get("video", False) else frame
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break
    # resize the frame, blur it, and convert it to the HSV
    # color space
    # (2705x1525)
    frame = imutils.resize(frame, height=1525)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "orange", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            ######## DETECTING ########

            # line1
            if (
                ball_isout == True
                and center[0] > (902 - CHECK_RANGE)
                and center[0] < 902
            ):
                if LINE1_TOGGLE == True:
                    LINE1_TOGGLE = False
                else:
                    LINE1_TOGGLE = True
                ball_isout = False

                LINE1_XY = center
                print("line1 detect")
                print(center)

            # line2
            if (
                ball_isout == True
                and center[0] > 1802
                and center[0] < (1802 + CHECK_RANGE)
            ):
                if LINE2_TOGGLE == True:
                    LINE2_TOGGLE = False
                else:
                    LINE2_TOGGLE = True
                ball_isout = False

                LINE2_XY = center
                print("line2 detect")
                print(center)

            # isout detector
            if (
                (center[0] < (902 - CHECK_RANGE))
                or (center[0] > 902 and center[0] < 1802)
                or (center[0] > 1802 + CHECK_RANGE)
            ):
                ball_isout = True

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
                print(FINAL_XY)

    # update the points queue
    pts.appendleft(center)
    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # center line drawing
    cv2.line(frame, (1352, 0), (1352, 1525), (255, 0, 0), 5)

    # line1 drawing
    if LINE1_TOGGLE == True:
        cv2.rectangle(frame, (902 - CHECK_RANGE, 0, CHECK_RANGE, 1525), (0, 255, 0), 5)
    else:
        cv2.rectangle(frame, (902 - CHECK_RANGE, 0, CHECK_RANGE, 1525), (0, 0, 255), 5)

    # line2 drawing
    if LINE2_TOGGLE == True:
        cv2.rectangle(frame, (1802, 0, CHECK_RANGE, 1525), (0, 255, 0), 5)
    else:
        cv2.rectangle(frame, (1802, 0, CHECK_RANGE, 1525), (0, 0, 255), 5)

    # predict line drawing
    if line_ison == True:
        cv2.line(frame, FINAL_XY, LINE2_XY, (255, 0, 0), 5)

    # show the frame to our screen
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()
# otherwise, release the camera
else:
    vs.release()
# close all windows
cv2.destroyAllWindows()
