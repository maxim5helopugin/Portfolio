"""Video analysis tool to examine frames, draw bounding boxes around regions of interest, and create data files that
define an object to be learned.

Provides an interface for users to graphically label video data to be used in vision training.

Input: raw sample video

Output: labeled image data
    region containing object of interest
    cropped object
    image frame

Sample files:
    positive info: pos_image.jpg 0 x y w h
    negative info: neg_image.jpg

Controls:
    Click and drag to draw rectangular region surrounding object
    Drag corners to resize
    Drag center to move region

    w       move up
    a       move right
    s       move down
    d       move right
    Escape  Quit
    Space   Next frame
    c       Crop positive sample
"""
import cv2
import numpy as np
import sys
import os
import argparse

from image_utils import *


# path to video for analysis
video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/IDEA2/Orange/orangeA1.mp4"
# video = 'C:/Users/seanr/OneDrive/UAV/UAV/prey/Idea2/Orange/orange_9am.MP4'
# path to save data, should end in slash /
path = 'train/data/'

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--video", help="Location of video file to analyze")
parser.add_argument("-p", "--path", help="Path to save images")
args = parser.parse_args()

if args.video:
    video = args.video

# uses name of video file as prefix for saved data
name = video.split('/')[-1].split('.')[0]

# Globals
roi = []            # points defining region of interest- two opposite corners of rectangle
drawing = False     # dragging rectangle corner
drawn = False       # both points set
temp_pt = None      # dragging point
drag = False        # moving roi rectangle
origin = None       # origin of dragging movement
img = None


def on_mouse(event, x, y, flags, param):
    """Handles mouse drawing actions in GUI."""
    global roi, drawing, drawn, temp_pt, drag, origin, img
    pt = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN:
        if drawn:
            i = on_point(pt, roi)
            if i >= 0:
                pt = roi[i - 1]
            elif on_point(pt, [midpoint(roi[0], roi[1])], 9) == 0:
                    drag = True
                    origin = pt
                    return
        roi = [pt]
        drawing = True
        temp_pt = pt
    elif event == cv2.EVENT_LBUTTONUP:
        if not drag:
            roi.append(pt)
        drawing = False
        drawn = True
        drag = False
        temp_pt = None
    elif event == cv2.EVENT_MOUSEMOVE:
        if drag:
            x_offset = pt[0] - origin[0]
            y_offset = pt[1] - origin[1]
            roi = shift(roi, x_offset, y_offset, 1)
            origin = pt
        elif drawing:
            temp_pt = pt
    draw(img)


def draw(image):
    """Draws the bounding box and other elements for GUI."""
    img = image.copy()
    if len(roi) == 2:
        cv2.rectangle(img, roi[0], roi[1], (0, 255, 0), 1)
        center = midpoint(roi[0], roi[1])
        cv2.circle(img, center, 9, (255, 255, 255), 2)
    if drawing and temp_pt:
        cv2.rectangle(img, roi[0], temp_pt, (255, 0, 0), 1)
    for pt in roi:
        cv2.circle(img, pt, 5, (255, 255, 255), -1)
    cv2.imshow('Video analysis', img)


def valid_roi(roi, img):
    """Ensure that the selected ROI is within the bounds of the image."""
    if len(roi) < 2:
        return False
    size = img.shape[:-1]
    for pt in roi:
        if not in_region(pt, [(0, 0), (size[1], size[0])]):
            return False
    return True


def crop(roi, img, time=None, path='train/pos/cropped/'):
    """Saves a cropped version of the region of interest from an image."""
    global name
    if not os.path.exists(path):
        os.makedirs(path)
    cropped = img[roi[0][1]:roi[1][1], roi[0][0]:roi[1][0]]
    filename = path + name + str(time) + '.jpg'
    cv2.imwrite(filename, cropped)


def sample_positive(img, roi, time=None, path='train/pos/'):
    """Save and write image name, num objects (1), top left coordinates, width, and height."""
    global name
    x, y, w, h = cv2.boundingRect(np.array(roi))
    roi = [(x, y), (x + w, y + h)]
    if not os.path.exists(path):
        os.makedirs(path)
    filename = path + name + str(time) + '.jpg'
    cv2.imwrite(filename, img)
    crop(roi, img, time, path + 'cropped/')
    line = "%s 1 %d %d %d %d\n" % (filename, x, y, w, h)
    print(line)
    if w < 0 or h < 0:
        print("NOT ADDED: incorrect region dimensions.")
        return
    with open('pos_info_' + name + '.dat', 'a') as file:
        file.write(line)


def sample_negative(img, path='train/neg/', time=None):
    """DEPRECIATED: saves a negative image that doesn't contain the object, and write this information to a file."""
    global name
    if not os.path.exists(path):
        os.makedirs(path)
    filename = path + 'neg' + name + str(time) + '.jpg'
    cv2.imwrite(filename, img)
    with open('neg_info.txt', 'a') as file:
        file.write(filename + '\n')


def examine(line):
    """"Displays image and outlines region of interest"""
    content = line.split()
    filename = content[0]
    x, y, w, h = [int(item) for item in content[2:]]
    img = cv2.imread(filename)
    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 1)
    cv2.imshow('Validate image', img)
    print(content[0])

    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'):
        sys.exit(0)


def main(path="train/"):
    global img, roi

    # VIDEO capture and GUI setup
    iw = "Video analysis"  # image window name
    vid = cv2.VideoCapture(video)
    cv2.namedWindow(iw)
    cv2.setMouseCallback(iw, on_mouse)

    # while vid.next():
    while True:
        ret, img = vid.read()
        if ret is False:
            break
        img_copy = img.copy()
        t = int(vid.get(cv2.CAP_PROP_POS_FRAMES))

        cv2.imshow(iw, img)
        # Keyboard control
        while True:
            key = cv2.waitKey(0) & 0xFF
            draw(img)
            if key == 27:     # escape
                break
            elif key == ord('w'):
                roi = shift(roi, 0, -1)
                draw(img)
            elif key == ord('a'):
                roi = shift(roi, -1, 0)
                draw(img)
            elif key == ord('s'):
                roi = shift(roi, 0, 1)
                draw(img)
            elif key == ord('d'):
                roi = shift(roi, 1, 0)
                draw(img)
            elif key == ord('c'):   # crop and save positive image
                if valid_roi(roi, img):
                    sample_positive(img_copy, roi, time=t, path=path+'pos/')
                else:
                    print("Invalid ROI")
            elif key == ord('n'):   # write negative sample
                sample_negative(img_copy, time=t, path=path+'neg/')
            else:
                break
        t += 1
        if key == 27:  # escape quits
            break

    vid.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(path=path)
