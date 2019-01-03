from __future__ import division, print_function

import cv2
from image_utils import *
import sys
import random

from video import Video
from utils.timer import Timer
import utils.histograms as hist


class Sample:
    """Labeled and bounded image frame"""
    def __init__(self, filename, x, y, w, h):
        # self.img = cv2.imread(filename)
        self.img_file = filename
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def region(self):
        """Returns two corner points of rectangular region."""
        return [(self.x, self.y), (self.x + self.w, self.y + self.h)]


class Trainer:
    """Learn features from labelled data, then train, test, and filter those features."""
    def __init__(self, extractor=None, matcher=None, n=None):
        self.feature_set = []
        self.image_set = []
        self.extractor = extractor
        self.matcher = matcher
        self.data = []
        self.n = n
        self.feature_scores = None

        self.downsample = True      # Scale down image and corresponding region coordinates before processing

        # Color analysis
        self.hues = np.zeros(180)
        self.sats = np.zeros(256)
        self.vals = np.zeros(256)

    def set_detector(self, detector):
        self.extractor = detector.extractor
        self.matcher = detector.matcher
        self.detector = detector

    def load_data(self, info_file, n=None):
        """Load data file of object bounding boxes locations in sample images."""
        with open(info_file) as input:
            for i, line in enumerate(input):
                content = line.split()
                filename = content[0]
                x, y, w, h = [int(item) for item in content[2:]]
                if w + h < 50:
                    # print('Too small', filename)
                    continue

                if self.downsample:
                    x = int(x/2)
                    y = int(y/2)
                    w = int(w/2)
                    h = int(h/2)

                self.data.append(Sample(filename, x, y, w, h))
                if n is not None and i > n:
                    break

    def set_features(self, descriptors, extend=False, sample_n=None):
        """Set features to descriptors, updating the detector and optionally randomly sample a subset to save."""
        if sample_n is not None:
            print('(Trainer) features found: %d -> %d' % (len(self.feature_set), sample_n))
            if len(descriptors) > sample_n:
                descriptors = random.sample(descriptors, sample_n)

        if not extend:
            self.feature_set = []
        self.feature_set.extend(descriptors)
        self.detector.set_features(self.feature_set)

    def subsample_data(self, subsample=3):
        """Sample every nth sample"""
        original = len(self.data)
        if subsample > 1:
            indices = np.linspace(0, original - 1, int(original/subsample), dtype=np.uint32)
            self.data = np.array(self.data)[indices]
        print('(Trainer) Using %d/%d samples' % (len(self.data), original))

    def train_and_test(self, split=.5, subsample=1, randomize=True, show=False):
        """Split the data set randomly into training and testing sets. Train then evaluate on test set."""
        random.shuffle(self.data)
        s = int(len(self.data) * split)
        train_set = self.data[:s]
        self.data = self.data[s:]   # Remove train set from further testing

        self.detector.set_max_features(40)
        self.train(train_set, show=show)
        self.color_info()

        self.detector.set_max_features(900)
        self.evaluate(show=show)
        self.feature_selection()

    def train(self, train_set=None, show=False, color_analysis=True):
        """Collect and save features from labelled sample regions."""
        if show:
            iw = 'Train'
            cv2.namedWindow(iw)
            cv2.createTrackbar('Delay', iw, 0, 500, nothing)
            cv2.setTrackbarPos('Delay', iw, 1)

        if train_set is None:
            train_set = self.data
        print('(Trainer) Training on %d samples' % len(train_set))

        new_features = []

        for sample in train_set:
            img = cv2.imread(sample.img_file)
            if img is None:
                print('(Trainer) No img', sample.img_file)
                continue
            if self.downsample:
                img = cv2.pyrDown(img)

            h, w = img.shape[:2]
            mask = np.zeros((h, w, 1), dtype=np.uint8)
            p1, p2 = sample.region()
            mask = cv2.rectangle(mask, p1, p2, 255, -1)
            kp, des = self.detector.detect_features(img, mask)
            if des is not None:
                new_features.extend(des)
                if color_analysis:
                    self.add_color_sample([img[int(k.pt[1]), int(k.pt[0])] for k in kp])

            if show:
                img = draw_keypoints(img, kp, (0, 255, 0))
                # img = cv2.addWeighted(img, 1, np.dstack((mask, mask, mask)), 1, 0)
                cv2.imshow(iw, img)
                cv2.waitKey(cv2.getTrackbarPos('Delay', iw))

        print('(Trainer) Learned %d features' % len(new_features))
        self.set_features(new_features, extend=False)

    # Color analysis
    def add_color_sample(self, bgr):
        hsv = cv2.cvtColor(np.array(bgr).reshape(-1, 1, 3), cv2.COLOR_BGR2HSV)
        hsv = hsv.reshape(-1, 3)
        hues = hsv[:, 0]
        sats = hsv[:, 1]
        vals = hsv[:, 2]
        for h in hues:
            self.hues[h] += 1
        for s in sats:
            self.sats[s] += 1
        for v in vals:
            self.vals[v] += 1
        # self.hues[hsv.reshape(3)[0]] += 1

    def color_info(self):
        cols = np.arange(self.hues.shape[0], dtype=np.uint16)
        cols2 = np.arange(self.vals.shape[0], dtype=np.uint16)
        # print('Hues')
        # print(np.dstack((cols, self.hues)).reshape(-1, 2))
        from utils.histograms import plot_histogram
        plot_histogram(self.hues, cols, title="Hue", max_x=180)
        plot_histogram(self.sats, cols2, title="Saturation", max_x=255)
        plot_histogram(self.vals, cols2, title="Value", max_x=255)

    def train_images(self, images=None, show=False, extend=False, border=26, show_histogram=False, randomize=True):
        """Extract features from a set of cropped images."""
        if images is None:
            images = [d.img for d in self.data]
        print('(Trainer) Training on %d images' % len(images))
        found_features = []
        for img in images:
            img = cv2.copyMakeBorder(img, top=border, bottom=border, left=border, right=border,
                                        borderType=cv2.BORDER_CONSTANT, value=[255, 255, 255])
            kp, des = self.detector.detect_features(img)
            if show:
                img2 = draw_keypoints(img, kp, flag=1)
                key = show_img(img2, 'training', delay=350)
                if key == 27:
                    show = False
            if des is not None:
                found_features.extend(des)

        self.set_features(found_features, extend=extend)
        print('(Trainer) Found %d -> %d features' % (len(found_features), len(self.feature_set)))
        # self.detector.set_features(self.feature_set)

        # if show_histogram:
        #     hsvs = np.array(hsvs).reshape(-1, 3)
        #
        #     hues = hsvs[:, 0]
        #     freq, thresh = hist.histogram(hues, 180, 180)
        #     hist.plot_histogram(freq, thresh, title='Hue', max_x=180)
        #
        #     sats = hsvs[:, 1]
        #     freq, thresh = hist.histogram(sats, 180, 180)
        #     hist.plot_histogram(freq, thresh, title='Saturation', max_x=180)
        #
        #     vals = hsvs[:, 2]
        #     freq, thresh = hist.histogram(vals, 180, 180)
        #     hist.plot_histogram(freq, thresh, title='HSV Values', max_x=180)
        #
        #     plt.show()

    def evaluate(self, show=False, data=None, subsample=1):
        """Given a data set of labeled/bounded images, evaluate the detection algorithm's precision/recall."""
        if data is None:
            data = self.data
            if subsample < 1:
                data = random_sample(data, subsample)

        if show:
            iw = 'Evaluate'
            cv2.namedWindow(iw)
            cv2.createTrackbar('Delay', iw, 0, 5, nothing)
            cv2.setTrackbarPos('Delay', iw, 1)

        feature_set_length = len(self.matcher.features)
        self.feature_scores = np.zeros((feature_set_length, 2), dtype=np.uint32)    # descriptor true +s and false +s
        print('(Trainer) Evaluating %d features on %d samples' % (feature_set_length, len(self.data)))

        t = Timer('Evaluation')
        total_positives = 0   # detected keypoints in region, used for recall calculation
        for sample in data:
            img = cv2.imread(sample.img_file)
            if img is None:
                print('(Trainer) No img', sample.img_file)
                continue
            if self.downsample:
                img = cv2.pyrDown(img)

            t.start()
            kp, des = self.detector.detect_features(img)
            for k in kp:
                if in_region(k.pt, sample.region()):
                    total_positives += 1

            matches = self.detector.match_descriptors(des, self.feature_set)
            inliers = []
            outliers = []
            inner_kp = []
            outer_kp = []

            for m in matches:
                pt = kp[m.queryIdx].pt
                if in_region(pt, sample.region()):
                    inliers.append(m.trainIdx)
                    inner_kp.append(m.queryIdx)
                else:
                    outliers.append(m.trainIdx)
                    outer_kp.append(m.queryIdx)
            self.score(inliers, outliers)

            t.end()

            # inliers = [m for m in matches if in_region(kp[m.queryIdx].pt, sample.region())]
            # outliers = [m for m in matches if m not in inliers]
            # self.score([m.trainIdx for m in inliers], [m.trainIdx for m in outliers])
            if show:
                # print("Total: %d, Inliers: %d, Outliers: %d" % (len(matches), len(inliers), len(outliers)))
                inliers = [kp[i] for i in inner_kp]
                outliers = [kp[i] for i in outer_kp]
                img = draw_keypoints(img, inliers, color=(0, 255, 0))
                img = draw_keypoints(img, outliers, color=(0, 0, 255))

                # img = draw_keypoints(img, matching_keypoints(inliers, kp), color=(0, 255, 0))
                # img = draw_keypoints(img, matching_keypoints(outliers, kp), color=(0, 0, 255))

                img, xx, yy = self.detector.detection_offset(img, sample.region(), confidence=len(inliers))
                pt1 = int(sample.x), int(sample.y)
                pt2 = pt1[0] + sample.w, pt1[1] + sample.h
                img = cv2.rectangle(img, pt1, pt2, (255, 255, 255), 1)
                # img = cv2.rectangle(img, sample.region()[0], sample.region()[1], (255, 0, 0), 1)

                delay = cv2.getTrackbarPos('Delay', iw)
                cv2.imshow(iw, img)
                key = cv2.waitKey(delay)
                # key = show_img(img, iw, delay)
                if key == 27:
                    cv2.destroyAllWindows()
                    show = False

        if show:
            cv2.destroyAllWindows()
        t.output()

        return self.evaluation_scores(total_positives)

    def score(self, inliers, outliers):
        """Record detection results."""
        for i in inliers:   # true positives
            self.feature_scores[i, 0] += 1
        for o in outliers:  # false positives
            self.feature_scores[o, 1] += 1

    def feature_selection(self, min_true=3, false_rate=2.5):
        """Given feature scores, filter to keep only best features.
        # min_true:   Minimum true positives per feature
        # false_rate: Maximum false positive ratio
        """
        indices = []
        for i, [good, bad] in enumerate(self.feature_scores):
            if good > min_true and good * false_rate > bad:
                indices.append(i)

        total_length = len(self.feature_set)
        self.feature_set = np.array(self.feature_set)[indices]
        print('(Trainer) Reduce features from %d to %d' % (total_length, len(self.feature_set)))
        self.feature_scores = self.feature_scores[indices]
        # np.set_printoptions(threshold=np.nan)
        print('(Trainer) Feature scores: ', self.feature_scores)

        self.feature_set, self.feature_scores = self.cross_check(self.feature_set, self.feature_scores)
        print('(Trainer) Feature scores: ', self.feature_scores)

        self.detector.set_features(self.feature_set)

    def cross_check(self, features, scores, dist=16, min_score=15):
        """Remove similar features."""
        features = np.unique(features, axis=0)
        half = int(len(features) / 2)
        f1 = features[:half]
        f2 = features[half:]

        redundants = []
        matches = self.detector.match_descriptors(f1, f2)
        for m in matches:
            if m.distance < dist:
                if scores[m.queryIdx][0] > scores[m.trainIdx + half][0]:
                    # check if it's actually a good feature
                    if scores[m.trainIdx + half][0] < min_score:
                        redundants.append(m.trainIdx + half)
                else:
                    if scores[m.queryIdx][0] < min_score:
                        redundants.append(m.queryIdx)

        if len(redundants) > 0:
            redundants.sort(reverse=True)
            print(redundants)
            features = list(scores)
            scores = list(scores)
            for i in redundants:
                features.pop(i)
                scores.pop(i)
            print('(Trainer) Removed %d redundant features' % len(redundants))
        scores = np.array(scores)
        return np.vstack((f1, f2)), scores

    def evaluation_scores(self, total_positives):
        # Statistics
        true_positives = sum(self.feature_scores[:, 0])
        false_positives = sum(self.feature_scores[:, 1])
        precision = true_positives / (true_positives + false_positives)
        recall = true_positives / total_positives
        print('(Trainer) Feature scores:', self.feature_scores)
        print('(Trainer) Precision: %.5f, %d/%d' % (precision, true_positives, false_positives))
        print('(Trainer) Recall: %.5f, %d/%d' % (recall, true_positives, total_positives))
        return precision, recall

    def save_features(self, filename='trained_set'):
        idx = 0
        ext = '.dat'
        import os.path
        while os.path.exists(filename + idx + ext):
            idx += 1
        self.detector.save_features(filename + idx + ext)

    def save_features(self, name='features', path='data/'):
        self.detector.save_features(name=name, path=path)

    def load_features(self, file, sample=1):
        self.detector.load_features(file, sample=sample)


def index_mask(a, indices):
    """Utility function return only elements in list a with index in indices."""
    return [a[i] for i in indices]


def analyze_colors(colors, bins=50, cvt=cv2.COLOR_BGR2HSV):
    print('Colors', colors)
    channels = len(colors[0])
    colors = np.array(colors).reshape(-1, channels)

    channel_ranges = [180, 180, 255]
    titles = ['Channel 1', 'Channel 2', 'Channel 3']

    for channel in range(channels):
        col = colors[:, channel]
        freq, thresh = hist.histogram(col, bins, channel_ranges[channel])
        hist.plot_histogram(freq, thresh, title=titles[channel])
    plt.show()


def random_sample(array, s=0.5):
    """Randomly sample s% of array."""
    mask = np.random.choice([False, True], len(array), p=[1 - s, s])
    return array[mask]
    # sample_n = items.shape[0] * s
    # items = random.sample(list(items), sample_n)
    # return np.array(items)
