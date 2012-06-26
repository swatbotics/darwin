#!/usr/bin/env python

from matplotlib import pyplot as plt
import subprocess
import sys

TAGTEST_BIN = "../build/bin/tagtest"

TEST_IMAGES = [
    "images/darwincam/1-debug_0_0.png",
    "images/darwincam/2-debug_0_0.png",
    "images/darwincam/3-debug_0_0.png",
    "images/darwincam/4-debug_0_0.png",
    "images/darwincam/5-debug_0_0.png",
    "images/darwincam/6-debug_0_0.png",
    "images/darwincam/7-debug_0_0.png",
]

def get_image_counts(flags):
    p = subprocess.Popen([TAGTEST_BIN] + flags + TEST_IMAGES,
                         stdout=subprocess.PIPE)
    output = p.communicate()[0]
    return parse_detection_counts(output)

def parse_detection_counts(tagtest_output):
    lines = tagtest_output.split('\n')
    return tuple(int(line.split()[1]) for line in lines
                 if line and line.split()[0] == 'Got')

def main():
    a_values = range(0, 160 + 1, 1)
    ideal_counts = get_image_counts(['-Rx'])
    test_counts = [get_image_counts(['-Rx', '-D', '-a %d' % a])
                   for a in a_values]
    all_data = zip(range(len(ideal_counts)), ideal_counts, zip(*test_counts))
    for index, ideal, actuals in all_data:
        plt.plot(a_values, [ideal] * len(a_values), 'k:')
        plt.plot(a_values, actuals,
                 label='Image #%d (%d)' % (index + 1, ideal))
    plt.ylim(0, max(ideal_counts) + 5)
    plt.xlabel("Values for theta threshold, K_D")
    plt.ylabel("Number of detections")
    plt.title("Number of detections vs. theta threshold values")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
