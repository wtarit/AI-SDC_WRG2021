import cv2
import numpy as np

DIM = (640, 480)
K = np.array([[305.71335049160973, 0.0, 330.5816390262722], [0.0, 306.2497590425206, 200.71141718207798], [0.0, 0.0, 1.0]])
D = np.array([[-0.034111560999105804], [0.006556441877070668], [-0.023339608690602854], [0.013723480642469804]])
def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    undistort("captured_image/5.jpg")
