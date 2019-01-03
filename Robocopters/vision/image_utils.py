"""Utilities for image processing."""

import cv2
import math
import numpy as np

# COLOR

def to_gray(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return gray


def color_filter(img, kp, hue_range):
    """Return only keypoints with hue values in specified range."""
    pts = [k.pt for k in kp]
    filtered = []
    for i, p in enumerate(pts):
        x, y = p
        bgr = img[int(y), int(x)]
        hue = bgr_to_hue(bgr)
        if hue_range[0] <= hue <= hue_range[1]:
            filtered.append(kp[i])
    return filtered


def color_range_filter(img, kp, low_bound, high_bound, cvtColor=None):
    """Filter keypoints by color range."""
    filtered_kp = []
    if len(kp) < 1:
        return filtered_kp
    colors = np.array([img[int(k.pt[1]), int(k.pt[0])] for k in kp]).reshape(-1, 1, 3)
    if cvtColor is not None:
        colors = cv2.cvtColor(colors, cvtColor)
    res = np.array(cv2.inRange(colors, np.array(low_bound), np.array(high_bound)))
    for i in range(res.size):
        if res[i] == 255:
            filtered_kp.append(kp[i])
    return filtered_kp


def bgr_to_hue(bgr):
    """"Convert BGR color to HSV and return hue component."""
    return bgr_to_hsv(bgr)[0]
    # rgb = bgr[::-1]
    # hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)
    # return hsv[0][0][0]


def bgr_to_hsv(bgr):
    """"Convert BGR color to HSV."""
    hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)
    return hsv[0][0]


# IMAGE FUNCTIONS

def threshold(img, t):
    """Intensity thresholding."""
    return np.where(img < t, 0, 255)


def hue_range_thresh(img, low_h, high_h):
    """Threshold image to hue values range ([0, 180], [0, 180])"""
    h, w, d = img.shape
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    binary = np.zeros((h, w))
    for i in range(w):
        for j in range(h):
            if low_h < hsv[j, i, 2] < high_h:
                binary[j, i] = 1
    return binary


def show_img(img, title="Image", delay=0):
    """Show image with OpenCV."""
    img = np.array(img, dtype='uint8')
    # cv2.namedWindow(title)
    cv2.imshow(title, img)
    key = cv2.waitKey(delay) & 0xFF
    if key == ord('p'):  # save
        num = 0
        filename = 'saved/%s%d' % (title, num)
        import os
        while os.path.isfile(filename + '.jpg'):
            num += 1
            filename = 'saved/%s%d' % (title, num)
        save_img(img, filename)
    cv2.destroyAllWindows()
    return key


def save_img(img, filename):
    path = '/'.join(filename.split('/')[:-1])
    ext = filename.split('.')[-1]
    import os
    if not os.path.exists(path):
        os.makedirs(path)
    if ext not in ['jpg', 'png']:
        filename += '.jpg'
    cv2.imwrite(filename, img)
    print('Write file to', filename)


def list_files(root, ext=['jpg', 'png']):
    """List the files, including """
    import os
    list = []
    for subdir, dirs, files in os.walk(root):
        for f in files:
            path = os.path.join(subdir, f)
            extension = path.split('.')[-1]
            if extension in ext:
                list.append(path)
    return list


def load_images(files, fcn=None):
    images = []
    for file in files:
        img = cv2.imread(file)
        if fcn:
            img = fcn(img)
        images.append(img)
    return images


def load_image_folder(directory):
    files = list_files(directory)
    return load_images(files)


def scale_down(img, max_width, max_height):
    w, h = img.shape[:2]
    if w < max_width and h < max_height:
        return img
    if w > h:
        factor = w / max_width
    else:
        factor = h / max_height
    return cv2.resize(img, fx=factor, fy=factor, interpolation=cv2.INTER_AREA)


def reverseBGR(image):
    """Reverses color encoding scheme between RGB (OpenCV cv2) and BGR (Matplotlib plt) of an image."""
    return image[:, :, ::-1]


# POINT FUNCTIONS

def box_to_region(x, y, w, h):
    return [(x, y), (x + w, y + h)]


def inside(pt, center, radius):
    """Determine if a point lies within a circle."""
    x, y = center
    return (pt[0] <= x - radius <= x + radius) and (pt[1] <= y - radius <= y + radius)


def shift(set, dx, dy, step=2):
    """Shift a set of points."""
    return [(x + dx*step, y + dy*step) for (x, y) in set]


def centroid(pts):
    n = len(pts)
    x = 0
    y = 0
    for pt in pts:
        x += pt[0]
        y += pt[1]
    x /= n
    y /= n
    return x, y


def outlier_filter(kp, min_dist):
    pts = [k.pt for k in kp]
    min_dists = min_distances(pts)
    idxs = [i for i, d in enumerate(min_dists) if d < min_dist]
    return [k for i, k in enumerate(kp) if i in idxs]


def min_distances(pts):
    num_pts = len(pts)
    min_dists = [9000]*num_pts
    for i in range(num_pts):
        for j in range(num_pts):
            if i == j:
                continue
            d = dist(pts[i], pts[j])
            if d < min_dists[i]:
                min_dists[i] = d
    return min_dists


def inlier_fitler(kp, min_dist, min_inliers):
    """Given a set of (key)points, returns the indices of those which have min_inliers neighbors within min_dist pixels."""
    pts = [k.pt for k in kp]
    dists = distances(pts)
    inliers = []
    for i, pt in enumerate(pts):
        neighbors = 0
        for d in dists[i]:
            if d <= min_dist:
                neighbors += 1
        if neighbors > min_inliers + 1:
            inliers.append(kp[i])
    return inliers


def distances(pts):
    num_pts = len(pts)
    dists = np.zeros((num_pts, num_pts))
    for i in range(num_pts):
        for j in range(num_pts):
            if i == j:
                continue
            dists[i, j] = dist(pts[i], pts[j])
    return dists


def midpoint(a, b):
    return int((a[0] + b[0])/2), int((a[1] + b[1])/2)


def in_region(pt, region):
    (x1, y1), (x2, y2) = bound(region)
    x, y = pt
    return x1 <= x <= x2 and y1 <= y <= y2


def bound(pts):
    """Create a bounding region around a list of points"""
    if len(pts) < 2:
        return None
    x, y, w, h = cv2.boundingRect(np.array(pts))
    return [(x, y), (x + w, y + h)]


def within(x, a, b):
    return a <= x <= b or a >= x >= b


def dist(a, b):
    """Returns the Euclidian distance between two points."""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def on_point(new_pt, set, thresh=5):
    """Detect if new point matches a point in a set of points."""
    for i, pt in enumerate(set):
        if dist(new_pt, pt) <= thresh:
            return i
    return -1


# DRAW FUNCTIONS


def draw_matches(img, matches, kp, color=(255, 0, 0)):
    img = draw_keypoints(img, matching_keypoints(matches, kp), color)
    return img


def matching_keypoints(matches, kp):
    return [kp[m.queryIdx] for m in matches]


def draw_keypoints(img, kp, color=(255, 0, 0), flag=0, imgidx=False):
    flags = [cv2.DRAW_MATCHES_FLAGS_DEFAULT, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS] # 1 shows keypoint scale 
    cv2.drawKeypoints(img, kp, img, color=color, flags=flags[flag])
    return img


def draw_points(img, pts, color=(0, 255, 0), thickness=1):
    for pt in pts:
        x, y = pt
        img = cv2.circle(img, (x, y), 3, color, thickness=thickness)
    return img


# VIDEO FUNCTIONS

def skip(video, seconds):
    ms = seconds * 1000
    video.set(cv2.CAP_PROP_POS_MSEC, ms)


def nothing(x):
    pass

