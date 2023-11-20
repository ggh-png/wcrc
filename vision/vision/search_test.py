import cv2
import numpy as np

img = cv2.imread("/home/ingyu/ros2_ws/src/wcrc/vision/vision/test.png") # 배경이미지
#img = cv2.imread("normal.jpg") # 배경이미지
imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # 배경이미지를 흑백으로 변환
target = cv2.imread("/home/ingyu/ros2_ws/src/wcrc/vision/vision/images/star.png", cv2.IMREAD_GRAYSCALE) # 찾을 이미지. 불러올때부터 흑백


w, h = target.shape[::-1] # 타겟의 크기값을 변수에 할당

res = cv2.matchTemplate(imgGray, target, cv2.TM_CCOEFF_NORMED)
threshold = 0.5 # 0~1의 값. 높으면 적지만 정확한 결과. 낮으면 많지만 낮은 정확도.
loc = np.where(res>=threshold) # res에서 threshold보다 큰 값만 취한다.
for pt in zip(*loc[::-1]):
    cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2) # 결과값에 사각형을 그린다

cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()