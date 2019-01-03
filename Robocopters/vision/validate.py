"""Visually validate the correctness of training files."""

import cv2
import numpy
import sys


def main():
    input_file = 'data/pos_info.dat'
    input_file = 'pos_info_Courtyard_multi.dat'
    with open(input_file) as input:
        for line in input:
            examine(line)


def examine(line):
    """Displays image and outlines region of interest

    Input format: imageFile.jpg 1 x y w h"""
    content = line.split()
    filename = content[0]
    x, y, w, h = [int(item) for item in content[2:]]
    try:
        img = cv2.imread(filename)
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 1)
        cv2.imshow('Validate image', img)
    except:
        print('Failed: ', content[0])

    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'):
        sys.exit(0)


if __name__ == "__main__":
    main()
