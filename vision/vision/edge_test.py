import cv2
import math


# 사용할 변수들 미리 정의
FONT = cv2.FONT_HERSHEY_DUPLEX
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
FILTER_RATIO = 0.85


# 경계선을 가져오는 함수
def get_contours(img, min_area, is_simple=False):
    # 근사화 방식 Simple : 경계선의 꼭짓점 좌표만 반환
    if is_simple:
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 근사화 방식 None : 모든 경계선을 반환
    else:
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    result = []

    # 경계선 개수만큼 반복
    for cnt in contours:
        # 경계선의 너비가 최소 영역 이상일 때만 result 배열에 추가
        if cv2.contourArea(cnt) > min_area:
            result.append(cnt)

    return result


# 원형인지 여부를 반환하는 함수
def is_circle(cnt):
    cnt_length = cv2.arcLength(cnt, True)
    cnt_area = cv2.contourArea(cnt)

    # ratio가 1에 가까울수록 원형
    ratio = 4 * math.pi * cnt_area / pow(cnt_length, 2)

    if ratio > FILTER_RATIO:
        return True
    else:
        return False


# 꼭짓점을 그리는 함수
def draw_points(img, cnt, epsilon, color):
    cnt_length = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon * cnt_length, True)

    for point in approx:
        cv2.circle(img, (point[0][0], point[0][1]), 3, color, -1)


# 이미지 불러와서 필터링
img = cv2.imread("/home/ingyu/ros2_ws/src/wcrc/vision/vision/test.png")
filter_img = cv2.inRange(img, (0, 0, 0), (255, 150, 255))

# 경계선 가져오기
contours_simple = get_contours(filter_img, 50, True)
contours_none = get_contours(filter_img, 50, False)

# 텍스트 출력하고 경계선 그리기(simple)
simple_text = "contours count : " + str(len(contours_simple))
simple_img = cv2.putText(img.copy(), simple_text, (0, 25), FONT, 1, RED)
for cnt in contours_simple:
    cv2.drawContours(simple_img, cnt, -1, BLUE, 5)
    if is_circle(cnt):
        # 원형일 경우 빨간색으로 그리기
        draw_points(simple_img, cnt, 0.1, RED)
    else:
        # 원형이 아닐 경우 초록색으로 그리기
        draw_points(simple_img, cnt, 0.1, GREEN)

# 텍스트 출력하고 경계선 그리기(none)
none_text = "contours count : " + str(len(contours_none))
none_img = cv2.putText(img.copy(), none_text, (0, 25), FONT, 1, RED)
for cnt in contours_none:
    cv2.drawContours(none_img, cnt, -1, BLUE, 5)
    if is_circle(cnt):
        # 원형일 경우 빨간색으로 그리기
        draw_points(none_img, cnt, 0.1, RED)
    else:
        # 원형이 아닐 경우 초록색으로 그리기
        draw_points(none_img, cnt, 0.1, GREEN)


# 이미지 화면에 출력
cv2.imshow("origin image", img)
cv2.imshow("filter image", filter_img)
cv2.imshow("simple image", simple_img)
cv2.imshow("none image", none_img)
cv2.waitKey(0)