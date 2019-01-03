import cv2
from image_utils import *
from matplotlib import pyplot as plt


def hue_hist(img, pts, bins=16, normalize=True, show=False):
    """Histogram of hue values"""
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    fig = plt.figure()
    vals = [img[int(pt[1]), int(pt[0]), 0] for pt in pts]
    frequencies, thresholds = histogram(vals, bins)

    if normalize:
        x = np.sum(frequencies, dtype=np.float16)
        if x == 0:
            return None
        frequencies /= x

    if show:
        ax = fig.add_subplot(211)
        plot_histogram(frequencies, thresholds, ax, 'Hue')
        plt.show()

    return frequencies


def histogram(img, bins=128, max_val=255):
    """Create a pixel intensity histogram from a single channel image."""
    freqs = np.zeros(bins, dtype=np.float16)   # frequencies

    # array from 0 to 255 with values representing upper threshold of each bin
    thresholds = np.linspace(0, max_val, num=bins, dtype='int32')
    # img = img.flatten()
    for px in img:
        for i, t in enumerate(thresholds):
            if px <= t:
                freqs[i] += 1
                break
    print([(f, t) for f, t in zip(freqs,thresholds)])
    return freqs, thresholds


def plot_histogram(frequencies, thresholds, ax=None, title='Intensity', max_x=256):
    """Plots an intensity histogram of a grayscale image."""
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)

    ax.set_title('%s Histogram' % title)

    max_y = max(frequencies)

    # margin = max_x * .01   # space between bars
    margin = 0
    width = thresholds[1] - thresholds[0] + margin
    ax.bar(thresholds, frequencies, width, color='blue')

    plt.xlim([-1, max_x])
    plt.ylim([0, max_y])
    ax.set_xlabel(title)
    ax.set_ylabel('Quantity')
    plt.show()


def plt_show(title="", fullscreen=True):
    """Conveniently show image with matplotlib.pyplot in full screen window."""
    if title:
        fig = plt.gcf()
        fig.canvas.set_window_title(title)
        fig.suptitle(title, fontsize=20)
    plt.show()


def normalize(array):
    """Normalizes an array so its sum is 1."""
    print(len(array))
    x = np.sum(array, dtype=np.float16)
    if x == 0:
        print('aaa')
        return np.zeros_like(array)
    return array / x
