import cv2
import numpy as np
from image_utils import *
from video import Video
from training import Trainer
from detection import Detector, detection_offset
from utils.histograms import histogram

from utils.timer import Timer



video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Courtyard multi.mp4"
# video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Idea2/Red/red_mall2.MOV"
# video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/softball/field1 close.mp4"
# video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Idea2/Red/red_inside2.mp4"
# video = 'C:/Users/seanr/OneDrive/UAV/UAV/vid1.MOV'
# video = 'C:/Users/seanr/OneDrive/UAV/UAV/vid1.MOV'
# video = 'C:/Users/seanr/OneDrive/UAV/UAV/prey/Quadbox/quadbox_flight1.avi'


def region_colors(img, roi, convert=cv2.COLOR_BGR2HSV):
    (x, y), (x1, y1) = roi

    coordinates = []
    colors = []
    for i in range(x, x1):
        for j in range(y, y1):
            coordinates.append([i, j])
            colors.append(img[j, i])

    colors = np.array(colors).reshape(-1, 1, 3)
    if colors.size < 1:
        print('No colors')
        return
    colors = cv2.cvtColor(colors, convert)
    return colors.reshape((-1, 3))[:, 0]


def make_pdf(values, bins=24):
    frequencies, thresholds = histogram(values, bins, 180)
    x = np.sum(frequencies, dtype=np.float)
    frequencies /= x
    return frequencies


def sharpen(img):
    kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    img = cv2.filter2D(img, -1, kernel)
    return img


def auto_canny(img, sigma=0.33):
    x = np.median(img)
    lower = int(max(0, (1.0 - sigma) * x))
    upper = int(min(255, (1.0 + sigma) * x))
    edged = cv2.Canny(img, lower, upper)
    return edged


def nothing(a):
    pass


def preprocess(img):
    img = cv2.pyrDown(img)
    return img


def morph(img, erosion, dilation):
    """First erode the mask to remove noise, the dilate to fill missing points"""
    img = cv2.dilate(img, None, iterations=dilation)
    img = cv2.erode(img, None, iterations=erosion)
    return img


class CascadeDetector:
    def __init__(self, cascade_file=None):
        self.classifier = cv2.CascadeClassifier()
        if cascade_file is not None:
            self.classifier.load(cascade_file)

        self.prev_confidence = 0
        self.prev_detection = None

        # Parameters
        self.scale_factor = 1.3
        self.min_neighbors = 6
        self.output_weights = True

        self.max_detections = -1
        self.min_confidence = 0.3

    def detect(self, img):
        boxes, reject_levels, weights = self.classifier.detectMultiScale3(img, scaleFactor=self.scale_factor, minNeighbors=self.min_neighbors, outputRejectLevels=self.output_weights)
        if len(boxes) < 1:
            return img, 0

        # Sort detections by level weight (confidence)
        detections = zip(boxes, weights)
        detections = sorted(detections, key=lambda w: w[1], reverse=True)

        best_detection, confidence = detections[0]
        best_detection = box_to_region(*best_detection)
        hues = region_colors(img, best_detection)
        hues_pdf = make_pdf(hues)
        print('Hue PDF:', hues_pdf)

        draw_thickness = 3
        if self.prev_detection is not None:
            self.print('Prev detection: ', self.prev_detection)
            p1, p2 = self.prev_detection
            cv2.rectangle(img, p1, p2, (255, 0, 0), 2)

        for box, level in detections[:self.max_detections]:
            if level < self.min_confidence:
                break

            x, y, w, h = box
            b1 = (x, y)
            b2 = (x + w, y + h)
            if self.prev_detection is not None:
                if in_region(midpoint(b1, b2), self.prev_detection):
                    level += 10

            cv2.rectangle(img, b1, b2, (0, 255, 0), draw_thickness)
            cv2.putText(img, '%0.2f' % level, b1, cv2.FONT_HERSHEY_PLAIN, 1.6, (0, 0, 0))
            draw_thickness = 1

        self.prev_detection = best_detection
        self.prev_confidence = confidence
        # img = detection_offset(img, best_detection)
        return img, confidence

    def print(self, *text):
        string = '(Cascade) '
        print(string, text)




def main():
    vid = Video(video, 'Haar Cascade detection')
    vid.overlay = True

    cascade_file = 'data/idea_cascade.xml'
    detector = CascadeDetector(cascade_file)

    t = Timer('Cascade Detection')

    while vid.next():
        t.start()
        img = vid.img
        message = []
        img = preprocess(img)
        img, confidence = detector.detect(img)
        t.end()
        message.append('%0.2f' % confidence)
        vid.display(img, message)

    t.output()


if __name__ == "__main__":
    main()


