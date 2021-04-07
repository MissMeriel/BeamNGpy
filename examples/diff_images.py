import cv2
from skimage.measure import compare_ssim
import argparse
import imutils
import numpy as np

def main():
    pdir='C:/Users/merie/Documents/BeamNGpy-master/'
    #pattern = '1001'
    im1 = "{}{}.png".format(pdir, 'etk800_1001_before')
    im2 = "{}{}.png".format(pdir, 'etk800_1001_after')
    im1 = cv2.imread(im1)
    im2 = cv2.imread(im2)
    gray1 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    (score, diff) = compare_ssim(gray1, gray2, full=True)
    diff = (diff * 255).astype("uint8")
    print("SSIM: {}".format(score))
    num_diff_pixels = np.sum(im1 != im2)
    difference = cv2.absdiff(gray1, gray2)
    num_diff = np.sum(difference)
    print("Total pixels in image: {}".format(im1.size))
    print("number of different (gray) pixels: {}".format(num_diff))
    difference = cv2.absdiff(im1, im2)
    #num_diff = cv2.countNonZero(difference)
    num_diff = np.sum(difference)
    print("number of different (color) pixels: {}".format(num_diff))

if __name__ == '__main__':
    main()