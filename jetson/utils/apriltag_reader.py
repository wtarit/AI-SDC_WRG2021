import apriltag
import cv2

class apriltag_reader:
    def __init__(self, possible_tags):
        self.detector = apriltag.Detector()
        self.possible_tags = possible_tags

    def read(self, img):
        if len(img.shape) > 2:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(img)
        # print(results)
        tags = {}
        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            
            # check length on the top side of apriltag
            tagsize = ptB[0] - ptA[0]
            tagid = r.tag_id
            if tagid in self.possible_tags:
                tags[tagid] = tagsize
        return tags

if __name__ == "__main__":
    tag = apriltag_reader([131, 489])
    frame = cv2.imread("apriltag.jpg")
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(tag.read(frame))