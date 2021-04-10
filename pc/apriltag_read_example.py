import apriltag
import cv2
# from threaded_cam import ThreadedCamera

# cap = ThreadedCamera()

from threaded_cam import still_camera
cap = still_camera("apriltag.jpg")

detector = apriltag.Detector()
while True:
    img = cap.read()

    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    results = detector.detect(img_gray)
    print(results)
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the img
        # tagFamily = r.tag_family.decode("utf-8")
        tagid = str(r.tag_id)
        cv2.putText(img, tagid, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # print("[INFO] tag family: {}".format(tagFamily))

    cv2.imshow("frame", img)
    if cv2.waitKey(0) == ord('q'):
        break

cap.stop()