import cv2
import numpy as np

# Recommended params for cv2.calcPyrOpticalFlowLK
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 crit=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Params for sparse tracker
# lk_params2 = dict(winSize=(8, 8),
#                   maxLevel=3,
#                   crit=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


class Tracker:
    def __init__(self):
        self.lk_tracker = cv2.SparsePyrLKOpticalFlow_create(**lk_params)
        self.tracking = 0       # track for X frames, end at 0
        self.max_error = 40

        self.prev_frame = None
        self.prev_pts = None
        self.num_pts = 0
        self.confidence = 0

    def start_track(self, frame, pts, track_len, confidence):
        self.prev_frame = frame
        self.h, self.w = frame.shape[:2]
        self.prev_pts = np.array(pts, dtype=np.float32)
        self.num_pts = self.prev_pts.shape[0]
        self.tracking = track_len
        self.confidence = confidence

    def track(self, frame):
        """Use Lucas-Kanade Optical Flow to track moving points in successive images."""
        pts, status, error = self.lk_tracker.calc(self.prev_frame, frame, self.prev_pts, None)

        pts = pts[(status.reshape(-1,) == 1) & (error.reshape(-1,) < self.max_error)]
        x_bound = np.logical_and(pts[:, 0] >= 0, pts[:, 0] < self.w)
        y_bound = np.logical_and(pts[:, 1] >= 0, pts[:, 1] < self.h)
        inbounds = np.logical_and(x_bound, y_bound)
        pts = pts[inbounds]

        # for i, pt in enumerate(pts):
        #     if status[i] != 1:
        #         continue
        #     if error[i] > self.max_error:
        #         continue
        #     if 0 <= pt[0] < self.w and 0 <= pt[1] < self.h:
        #         good_pts.append(i)
        #     else:
        #         print('(Tracker) %s out of bounds' % pt)

        self.confidence -= 2 * (self.prev_pts.shape[0] - pts.shape[0])  # decrease confidence for lost points
        self.prev_pts = pts
        self.prev_frame = frame
        self.tracking -= 1
        return self.tracking, self.prev_pts
