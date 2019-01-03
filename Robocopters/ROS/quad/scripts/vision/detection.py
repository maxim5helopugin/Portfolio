"""Object detection and tracking using ORB feature keypoints and optical flow."""
from __future__ import division

import cv2
import numpy as np
from matching import BFMatcher
import pickle
from tracking import Tracker

# from utils.timer import Timer


def load_features(file, sample=1):
    """Load features from Pickled file"""
    with open(file, 'rb') as in_file:
        descriptors = pickle.load(in_file)
    if sample < 1:      # randomly sample (sample)% of the saved features
        n = len(descriptors) * sample
        import random
        descriptors = random.sample(descriptors, int(n))
    return descriptors


class Detector:
    """Performs feature extraction and matching."""
    def __init__(self, n_features=1000, t=38):
        self.matcher = BFMatcher()
        self.extractor = ORB_Extractor(n_features)
        self.descriptors = []

        self.t = t      # minimum distance metric for matching
        self.set_max_features(n_features)
        self.features = []      # main feature set

        # Image size and center coordinates
        self.h = 0
        self.w = 0
        self.origin_y = 0
        self.origin_x = 0

        self.profiles = []
        self.profile = None
        self.pid = 0

        self.features = []
        self.color_ranges = []
        self.colors = []
        self.weights = []

        self.orb_detect = True
        self.lk_track = True
        self.tracker = Tracker()

        self.time = 0
        self.prev_detection = 0         # time (frame) of previous detection

        self.DETECTION_THRESHOLD = 30   # Confidence threshold for detection
        self.LOCK_THRESHOLD = 38        # Confidence threshold to activate tracking

        self.CYCLE_INTERVAL = 10        # Frames without detection before switching profiles
        self.TRACK_INTERVAL = 8        # Frames to perform tracking of detected points (after hard lock)

    def load_profile(self, profile):
        """Load object information."""
        profile.features = load_features(profile.feature_file, 1)
        self.profiles.append(profile)
        self.profile = self.profiles[-1]
        print('(Detector) Loaded profile: ', self.profile.label)

    def detect(self, frame, mask=None):
        """Main function. Performs ORB detection and tracking."""
        img = frame.copy()
        kp_map = np.zeros(frame.shape[:2])
        self.time += 1
        if self.time_since_detection() > self.CYCLE_INTERVAL:
            self.cycle_profile()
            print('Detect mode: ', self.profile.label)

        # Output
        confidence = 0
        x = 0
        y = 0

        if self.tracker.tracking > 0:
            tracking, new_pts = self.tracker.track(frame)
            if new_pts.shape[0] > 1:
                # img = draw_points(img, new_pts, (0, 0, 255))
                if tracking == 0:
                    """Tracking epoch complete. Get the new point locations and add them to the keypoint map."""
                    for pt in new_pts:
                        kp_map[int(pt[1]), int(pt[0])] += 2
                    self.tracker.tracking = 0
                    self.orb_detect = True
                else:
                    """Skip ORB detection and continue tracking."""
                    self.orb_detect = False
                    pt1, pt2 = bound(new_pts)
                    confidence = self.tracker.confidence
                    img, x, y = self.detection_offset(img, [pt1, pt2], confidence)
            else:
                """Reset tracking and continue with ORB detection."""
                print('Reset tracking')
                self.tracker.tracking = 0
                self.orb_detect = True

        if self.orb_detect:     # Keypoint detection
            frame_kp, frame_des = self.detect_features(frame, mask)
            # for k in frame_kp:      # Draw all keypoints
            #     px, py = k.pt
            #     img[int(py), int(px)] = (0, 255, 0)

            kp, des = self.match(frame_kp, frame_des)
            # img = cv2.drawKeypoints(img, kp, img)
            if len(kp) > 3:
                pts = cv2.KeyPoint_convert(kp)
                pts = np.array([(int(pt[0]), int(pt[1])) for pt in pts]).reshape(-1, 2)
                img = draw_points(img, pts, (255, 0, 0))
                for i, pt in enumerate(pts):
                    kp_map[pt[1], pt[0]] = 1

                kp_colors = np.array([frame[pt[1], pt[0]] for pt in pts]).reshape(-1, 1, 3)
                kp_colors = cv2.cvtColor(kp_colors, cv2.COLOR_BGR2HSV)

                candidate_centers = []  # keypoints within color range
                for c in range(len(self.profile.colors)):
                    [low_bound, high_bound] = self.profile.color_ranges[c]
                    res = np.array(cv2.inRange(kp_colors, np.array(low_bound), np.array(high_bound)))
                    for i in range(res.size):
                        if res[i] == 255:
                            img = cv2.circle(img, (pts[i][0], pts[i][1]), 3, self.profile.colors[c], thickness=1)
                            candidate_centers.append(pts[i])
                            kp_map[pts[i][1], pts[i][0]] = self.profile.weights[c]

                # Find max window score
                if len(candidate_centers) > 0:
                    max_score = self.DETECTION_THRESHOLD
                    max_i = -1
                    win_size = 35
                    for i, pt in enumerate(candidate_centers):
                        score = region_sum(kp_map, pt, win_size)
                        if score > max_score:
                            max_score = score
                            max_i = i

                    # Draw box around best keypoint window
                    if max_i >= 0:
                        confidence = max_score
                        pt1 = candidate_centers[max_i]
                        pt2 = (pt1[0] + win_size, pt1[1] + win_size)
                        pt1 = (pt1[0] - win_size, pt1[1] - win_size)
                        img, x, y = self.detection_offset(img, [pt1, pt2], confidence=max_score)
                        if self.lk_track and self.tracker.tracking < 1 and max_score > self.LOCK_THRESHOLD:
                            """ Lock achieved: track points in the designated region. """
                            # Shrink region slightly
                            pt1 = np.add(pt1, 2)
                            pt2 = np.add(pt2, -2)
                            region_pts = inner_points(pt1, pt2, pts)
                            if len(region_pts) > 2:
                                # print('Tracking %d pts' % len(region_pts))
                                self.tracker.start_track(frame, region_pts, track_len=self.TRACK_INTERVAL, confidence=max_score)

        if confidence > self.DETECTION_THRESHOLD:
            self.prev_detection = self.time
        return img, confidence, x, y

    def set_max_features(self, n):
        """Maximum number of features to extract in frame."""
        self.extractor.extractor.setMaxFeatures(n)

    def time_since_detection(self):
        return self.time - self.prev_detection

    def cycle_profile(self):
        self.pid += 1
        if self.pid >= len(self.profiles):
            self.pid = 0
        self.profile = self.profiles[self.pid]
        self.prev_detection = self.time

    def print_info(self):
        d_size = self.extractor.extractor.descriptorSize()
        d_type = self.extractor.extractor.descriptorType()
        d_norm = self.extractor.extractor.defaultNorm()
        print('Type:', d_type, 'Size', d_size, 'Norm', d_norm)

    def detect_features(self, img, mask=None):
        kp, des = self.extractor.extract(img, mask)
        return kp, des

    def detect_keypoints(self, img, mask=None):
        """Return array of keypoint coordinates."""
        kp, _ = self.detect_features(img, mask)
        # kp = sorted(kp, key=lambda x: x.response, reverse=True)[:n]
        return np.array(cv2.KeyPoint_convert(kp)).reshape(-1, 1, 2)

    def match(self, kp, des, features=None, mask=None):
        """Extracts features from input image and matches them to known features."""
        matches = self.matcher.match(des, self.profile.features, t=self.t)
        matched_kp = matching_keypoints(matches, kp)
        matched_des = [des[m.queryIdx] for m in matches]
        return matched_kp, matched_des

    def match_descriptors(self, des1, des2):
        matches = self.matcher.match(des1, des2, t=self.t)
        return matches

    def match_descriptors_radius(self, des1, des2):
        matches = self.matcher.radiusMatch(des1, des2, maxDistance=self.t)
        return matches

    def save_features(self, name='features', path='data/'):
        """Output feature set to a file using Pickle."""
        import pickle
        import os
        num_features = len(self.descriptors)
        name += '_' + str(num_features)
        ext = '.dat'
        file_name = path + name + ext
        if not os.path.exists(path):
            os.makedirs(path)
        with open(file_name, 'wb') as out_file:
            pickle.dump(self.descriptors, out_file, protocol=2)
        out_file.close()
        print('(Detector) Saved %d features to %s' % (len(self.descriptors), file_name))

    def load_features(self, file, sample=1):
        """Load features from Pickled file"""
        self.descriptors = load_features(file, sample)
        self.matcher.features = self.descriptors
        print('(Detector) Loaded %d features from %s' % (len(self.descriptors), file))

    def set_features(self, feature_set):
        self.descriptors = feature_set
        self.matcher.features = feature_set

    def detection_offset(self, img, bounding_box, confidence=0, show=True):
        """Given a bounding box of the target, output the offset from the center."""
        if self.w == 0:     # Calibrate image dimensions
            self.h, self.w = img.shape[:2]
            self.origin_x = int(self.w / 2)
            self.origin_y = int(self.h / 2)
            if self.w == 0 or self.h == 0:
                return img, 0, 0

        pt1 = bounding_box[0]
        pt2 = bounding_box[1]
        center_x, center_y = midpoint(pt1, pt2)

        x = int((center_x / self.w) * 100)
        y = int((center_y / self.h) * 100)
        if show:
            img = cv2.rectangle(img, pt1, pt2, (0, 255, 0), 2)                                          # bounding box
            img = cv2.rectangle(img, (pt1[0], pt1[1] - 2), (pt1[0] + 39, pt1[1] - 17), (0, 0, 0), -1)   # text box
            text = str(confidence)
            if self.profile is not None:
                text = text + '  ' + self.profile.label
            img = cv2.putText(img, text, (pt1[0], pt1[1] - 3), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 255, 255))
            cv2.arrowedLine(img, (self.origin_x, self.origin_y), (center_x, center_y), color=(255, 255, 255),
                            thickness=1)  # offset
        return img, x, y


class ORB_Extractor:
    """"Extract image features"""
    def __init__(self, n=500):
        # self.extractor = cv2.ORB_create(scaleFactor=1.2, nfeatures=n, scoreType=cv2.ORB_FAST_SCORE)
        self.extractor = cv2.ORB_create(scaleFactor=1.3,
                                        nlevels=9,
                                        nfeatures=n,
                                        scoreType=cv2.ORB_HARRIS_SCORE)
        # self.extractor.setEdgeThreshold(40)
        # self.extractor.setMaxFeatures(n)

    def extract(self, img, mask=None):
        return self.extractor.detectAndCompute(img, mask)


def region_sum(mat, pt, width):
    """Sum of data values in array within window surrounding points."""
    x, y = pt
    h, w = mat.shape[:2]
    min_y = max(int(y - width), 0)
    max_y = min(int(y + width), h)
    min_x = max(int(x - width), 0)
    max_x = min(int(x + width), w)
    # if min_y < 0:
    #     min_y = 0
    # if min_x < 0:
    #     min_x = 0
    # if max_y > h:
    #     max_y = h
    # if max_x > w:
    #     max_x = w

    return int(np.sum(mat[min_y:max_y, min_x:max_x]))
    # sum = 0
    # for j in range(min_y, max_y):
    #     for i in range(min_x, max_x):
    #         sum += mat[j, i]
    # return int(sum)


def inner_points(vertex1, vertex2, pts):
    """Pick points within vertices of rectangle."""
    # inliers = []
    # for pt in pts:
    #     if in_region(pt, [vertex1, vertex2]):
    #         inliers.append(pt)
    # return inliers
    min_x = vertex1[0]
    min_y = vertex1[1]
    max_x = vertex2[0]
    max_y = vertex2[1]
    x_bound = np.logical_and(pts[:, 0] >= min_x, pts[:, 0] < max_x)
    y_bound = np.logical_and(pts[:, 1] >= min_y, pts[:, 1] < max_y)
    inbounds = np.logical_and(x_bound, y_bound)
    return pts[inbounds]


def in_region(pt, region):
    """Determine if point is within rectangular region."""
    (x1, y1), (x2, y2) = bound(region)
    x, y = pt
    return x1 <= x <= x2 and y1 <= y <= y2


def midpoint(a, b):
    return int((a[0] + b[0])/2), int((a[1] + b[1])/2)


def bound(pts):
    """Create a bounding region around a list of points"""
    if len(pts) < 2:
        return None
    x, y, w, h = cv2.boundingRect(np.array(pts))
    return [(x, y), (x + w, y + h)]


def draw_points(img, pts, color=(0, 255, 0), thickness=1):
    for pt in pts:
        x, y = pt
        img = cv2.circle(img, (x, y), 3, color, thickness=thickness)
    return img


def matching_keypoints(matches, kp):
    return [kp[m.queryIdx] for m in matches]

