""""""

import cv2
from sys import exit

import time

from image_utils import *


class Video:
    """Class for OpenCV video playback with GUI controls."""
    def __init__(self, video_file, iw='Video', delay=0, skip=1, fast=False, downsample=False):
        self.video = cv2.VideoCapture(video_file)
        if not self.video.isOpened():
            print('(Video) Failed to open video ', iw)
            exit(0)
        else:
            print('(Video) Opened', iw,  'successfully')

        self.iw = iw    # name of image window
        self.delay = delay  # slow playback speed, 0=paused
        self.streaming = False

        if type(video_file) == int:
            self.delay = 1
            self.streaming = True
            self.iw = 'Streaming ' + self.iw
            cv2.namedWindow(self.iw, cv2.WINDOW_FULLSCREEN)
            print('(Video)', self.iw)
        else:
            vid_name = video_file.split('/')[-1].split('.')[0]
            self.iw += ' ' + vid_name
            if not fast:
                """Remove trackbars and other Video features. Just plays video."""
                cv2.namedWindow(self.iw, cv2.WINDOW_FULLSCREEN)
            self.add_time_trackbar()
            self.skip(skip)
            if self.delay >= 0:
                # self.add_trackbar('Delay', 0, 10, self.set_delay)
                cv2.createTrackbar('Delay', self.iw, self.delay, 10, self.set_delay)
                self.set_delay(self.delay)
                cv2.setMouseCallback(self.iw, self.on_mouse)


        # GUI
        self.img = None     # image to shown
        self.downsample = downsample
        # self.width = int(self.video.get(cv2.CAP_PROP_FRAME_WIDTH))
        # self.height = int(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.canvas = None  # drawing/selection canvas
        self.active_canvas = False
        self.overlay = False

        self.time = 0       # time in frames from start of video

        # Trackbars


        # Video Recording
        self.recording = -1
        self.out_video = None

    def next(self):
        """Advances video feed. """
        ret, self.img = self.video.read()
        if not ret or not self.is_open():
            self.close()
            return False

        if self.downsample:
            self.img = cv2.pyrDown(self.img)
        if self.streaming:
            return True

        self.set_delay(self.delay)
        cv2.setTrackbarPos(self.iw, 'Delay', self.delay)
        self.time += 1
        self.update_timer()
        # print('(Video) Delay: ', cv2.getTrackbarPos('Delay', self.iw))
        return True

    def display(self, img, text=[]):
        """Display image and wait for command"""
        if self.recording > 0:
            self.out_video.write(img)
        if self.overlay:
            img = overlay(img, text)

        cv2.imshow(self.iw, img)
        if self.delay > 1:
            time.sleep(self.delay / 10)
        key = cv2.waitKey(self.delay) & 0xFF
        if key == 27:       # escape
            self.close()
        elif key == ord('p'):   # save current frame
            filename = 'saved/%s%d.jpg' % (self.iw, self.get_time())
            save_img(img, filename)
        elif key == ord('r'):    # record
            if self.recording < 0:
                self.recording = int(self.get_time())
                video_name = 'saved/' + self.iw + 'video' + str(self.recording) + '.avi'
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                h, w, _ = img.shape
                self.out_video = cv2.VideoWriter(video_name, fourcc, 7, (w, h))
                status = self.out_video.isOpened()
                print('(Video) Start recording', video_name, status)
            else:
                self.out_video.release()
                self.recording = -1
                print('(Video) Stop recording')

    def is_open(self):
        return self.video.isOpened()

    def close(self):
        if self.is_open():
            print('(Video) Closing %s at t=%d (%d)' % (self.iw, self.time, self.is_open()))
        if self.recording > 0:
            self.out_video.release()
            self.recording = -1
            print('(Video) Stop recording')
        self.video.release()
        cv2.destroyAllWindows()

    # GUI

    def add_trackbar(self, name, min, max, fcn=None, val=None):
        if val is None:
            val = min
        if fcn is None:
            fcn = nothing()
        cv2.createTrackbar(name, self.iw, min, max, fcn)
        cv2.setTrackbarPos(name, self.iw, val)

    def set_time(self, t):
        """On mouse callback for time trackbar."""
        self.video.set(cv2.CAP_PROP_POS_FRAMES, t)

    def update_timer(self, t=None):
        if t is None:
            t = self.get_time()
        cv2.setTrackbarPos('Time', self.iw, t)

    def get_time(self):
        return int(self.video.get(cv2.CAP_PROP_POS_FRAMES))

    def add_time_trackbar(self):
        """Add a trackbar to window to show/select video time."""
        frame_count = int(self.video.get(cv2.CAP_PROP_FRAME_COUNT))
        current_time = int(self.video.get(cv2.CAP_PROP_POS_FRAMES))
        self.add_trackbar('Time', 0, frame_count, fcn=self.set_time, val=current_time)

    def status(self):
        """Prints video information."""
        h = self.video.get(cv2.CAP_PROP_FRAME_HEIGHT)
        w = self.video.get(cv2.CAP_PROP_FRAME_WIDTH)
        fps = self.video.get(cv2.CAP_PROP_FPS)
        avi_ratio = self.video.get(cv2.CAP_PROP_POS_AVI_RATIO)
        print('(Video) FPS: %d | AVI Ratio: %d | Size: %d x %d' % (fps, avi_ratio, w, h))

    def skip(self, frames):
        new_time = self.get_time() + frames
        self.set_time(new_time)

    # def skip(self, seconds):
    #     """Untested version: skip ahead x seconds"""
    #     ms = seconds * 1000
    #     self.video.set(cv2.CAP_PROP_POS_MSEC, ms)

    def set_delay(self, delay):
        """Trackbar callback to set video speed."""
        self.delay = delay
        # if delay == 0 and self.active_canvas:
        #     self.canvas.img = self.img
        #     cv2.setMouseCallback(self.iw, self.canvas.on_mouse)
        # else:
        #     cv2.setMouseCallback(self.iw, self.on_mouse)

    def on_mouse(self, event, x, y, flags, param):
        # pause with space
        if event == cv2.EVENT_LBUTTONDOWN:
            bgr = self.img[y, x]
            hsv = bgr_to_hsv(self.img[y, x])
            print("HSV: %s  BGR: %s    x: %d, y: %d" % (hsv, bgr, x, y))
            self.pause()

    def pause(self):
        if self.delay == 0:
            self.set_delay(1)
            cv2.setTrackbarPos(self.iw, 'Delay', 1)
            cv2.waitKey(1)
        else:
            self.set_delay(0)
            cv2.setTrackbarPos(self.iw, 'Delay', 0)

    def add_canvas(self):
        self.canvas = Canvas(self.iw)
        self.active_canvas = True


class Canvas:
    """"Draw selection with mouse and keyboard controls on image/video."""

    def __init__(self, image_window):
        self.iw = image_window
        cv2.setMouseCallback(image_window, self.on_mouse)

        self.roi = []  # points defining region of interest- two opposite corners of rectangle
        self.drawing = False  # dragging rectangle corner
        self.drawn = False  # both points set
        self.temp_pt = None  # dragging point
        self.drag = False  # moving roi rectangle
        self.origin = None  # origin of dragging movement
        self.img = None

    def control(self, img):
        """Call before image display to control canvas drawing."""
        self.img = img
        while True:
            key = cv2.waitKey(0) & 0xFF
            self.draw()
            if key == 27:  # escape
                return -1
            elif key == ord('w'):
                self.roi = shift(self.roi, 0, -1)
                self.draw()
            elif key == ord('a'):
                self.roi = shift(self.roi, -1, 0)
                self.draw()
            elif key == ord('s'):
                self.roi = shift(self.roi, 0, 1)
                self.draw()
            elif key == ord('d'):
                self.roi = shift(self.roi, 1, 0)
                self.draw()
            elif key == 32:     # space
                if len(self.roi) == 2:
                    roi = self.roi
                    self.reset_drawing()
                    return roi
                else:
                    break
            else:
                break
        self.reset_drawing()
        return False

    def on_mouse(self, event, x, y, flags, param):
        """Handles mouse drawing actions in GUI."""
        pt = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.drawn:
                i = on_point(pt, self.roi)
                if i >= 0:
                    pt = self.roi[i - 1]
                elif on_point(pt, [midpoint(self.roi[0], self.roi[1])], 9) == 0:
                    self.drag = True
                    self.origin = pt
                    return
            self.roi = [pt]
            self.drawing = True
            self.temp_pt = pt
        elif event == cv2.EVENT_LBUTTONUP:
            if not self.drag:
                self.roi.append(pt)
            self.drawing = False
            self.drawn = True
            self.drag = False
            self.temp_pt = None
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drag:
                x_offset = pt[0] - self.origin[0]
                y_offset = pt[1] - self.origin[1]
                self.roi = shift(self.roi, x_offset, y_offset, 1)
                self.origin = pt
            elif self.drawing:
                self.temp_pt = pt
        self.draw()

    def reset_drawing(self):
        self.roi = []  # points defining region of interest- two opposite corners of rectangle
        self.drawing = False  # dragging rectangle corner
        self.drawn = False  # both points set
        self.temp_pt = None  # dragging point
        self.drag = False  # moving roi rectangle
        self.origin = None  # origin of dragging movement
        self.img = None

    def draw(self):
        """Draws the bounding box and other elements for GUI."""
        img = self.img.copy()
        if len(self.roi) == 2:
            cv2.rectangle(img, self.roi[0], self.roi[1], (0, 255, 0), 1)
            center = midpoint(self.roi[0], self.roi[1])
            cv2.circle(img, center, 9, (255, 255, 255), 2)
        if self.drawing and self.temp_pt:
            cv2.rectangle(img, self.roi[0], self.temp_pt, (255, 0, 0), 1)
        for pt in self.roi:
            cv2.circle(img, pt, 5, (255, 255, 255), -1)
        cv2.imshow(self.iw, img)


def overlay(img, text=[]):
    h, w, _ = img.shape
    overlay = np.zeros_like(img)
    midpt = (int(w / 2), int(h / 2))
    color = (255, 255, 255)
    crosshair_size = 40
    cv2.line(overlay, (midpt[0] + crosshair_size, midpt[1]), (midpt[0] - crosshair_size, midpt[1]), color)
    cv2.line(overlay, (midpt[0], midpt[1] + crosshair_size), (midpt[0], midpt[1] - crosshair_size), color)
    cv2.circle(overlay, midpt, 50, color, lineType=cv2.LINE_AA)

    box_w = 300
    box_h = 200
    overlay = cv2.rectangle(overlay, (w, 0), (w - box_w, box_h), (1, 1, 1), -1)
    # cv2.rectangle(overlay, (w, 0), (w - box_w, box_h), (100, 100, 100), -1)

    text_origin = [w - box_w + 8, 25]
    display_text = ['QUAD Alpha']
    display_text += text
    overlay = add_text(overlay, display_text, text_origin, text_color=color)

    return cv2.addWeighted(img, 1, overlay, .8, 0, img)


def add_text(img, text, origin, text_color=(255, 255, 255), bg=(0, 0, 0)):
    line_height = 40
    p1 = origin
    for t in text:
        img = cv2.putText(img, str(t), (origin[0], origin[1]), cv2.QT_FONT_NORMAL, .9, text_color, lineType=cv2.LINE_AA)
        origin[1] += line_height
    img = cv2.rectangle(img, (0, origin[1]), (p1[0], p1[1]), bg, -1)
    return img



def nothing():
    """Nothing"""
    pass
