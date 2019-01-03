import cv2
import numpy as np


class Matcher:
    def __init__(self):
        self.features = []
        self.draw = False
        self.matcher = None


class BFMatcher(Matcher):
    """Brute force matcher."""

    def __init__(self):
        Matcher.__init__(self)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    def match(self, des, features=None, t=40):
        """Use ratio test to filter 2-nearest-neighbor matches in known features."""
        # if features is None:
        #     features = self.features
        matches = self.matcher.match(des, np.array(features))
        # good_matches = ratio_test(matches, .85)
        # print("(Matcher) Matches: %d > %d (matching %d)" % (len(matches), len(good_matches), len(features)))
        good_matches = filter_matches(matches, t)
        return good_matches


# class TreeMatcher(Matcher):
#     """"Match features of query to known features using k nearest matches."""
#
#     def __init__(self):
#         Matcher.__init__(self)
#         self.matcher = flann_matcher()
#
#     def match(self, descriptors, features=None, n=None):
#         if features is None:
#             features = self.features
#         k = 2   # number of train features matched per query feature
#         matches = self.matcher.knnMatch(descriptors, np.array(features), k)
#
#         good_matches = ratio_test(matches, 0.99)
#         # print("(Matcher t) Matches: %d > %d" % (len(matches), len(good_matches)))
#         return filter_matches(good_matches, n)


def flann_matcher():
    """Uses FLANN (Fast Approximate Nearest Neighbor Search Library) matcher. This method is faster than brute force
    matchers for large data sets."""
    FLANN_INDEX_KDTREE = 0
    FLANN_INDEX_LSH = 6
    index_params = dict(algorithm=FLANN_INDEX_LSH,
                        table_number=12,
                        key_size=20,
                        multi_probe_level=2)
    search_params = dict(checks=40)  # 50
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    return flann


def filter_matches(matches, t=62):
    """Select the best matches by distance."""
    return [m for m in matches if m.distance < t]


def ratio_test(matches, ratio=.85):
    """Feature filtering method to reduce poor or ambiguous matches.

    ratio=0.8 suggested by Lowe
    If the ratio of the distances of the two best matches is less than threshold value, accept the closest match."""
    good_matches = []
    for (m, n) in matches:
        if m.distance < ratio * n.distance:
            good_matches.append(m)
    return good_matches
