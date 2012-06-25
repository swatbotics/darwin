#!/usr/bin/env python

from __future__ import division

import numpy as np
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
    "images/darwincam/8-debug_0_0.png",
    "images/darwincam/9-debug_0_0.png",
    "images/darwincam/10-debug_0_0.png",
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
    tests = [
        ('ideal', 'k', ['-Rx']),
        ('+ decimation', 'r', ['-Rx', '-D']),
        ('+ theta = 25', 'g', ['-Rx', '-D', '-a 25']),
        ('+ corners', 'b', ['-Rx', '-D', '-a 25', '-c']),
        ('+ subpix', 'y', ['-Rx', '-D', '-a 25', '-C']),
    ]
    all_data = [(label, color, get_image_counts(flags))
                for label, color, flags in tests]
    num_images = len(TEST_IMAGES)
    num_series = len(tests)
    inds = np.arange(num_images)
    width = (1.0 - 0.2) / num_series

    offset = 0
    legend_handles = []
    legend_labels = []
    max_count = 0
    for label, color, counts in all_data:
        max_count = max(counts + (max_count,))
        rectinfo = plt.bar(inds + offset, counts, width, color=color)
        legend_handles += [rectinfo[0]]
        legend_labels += [label]
        offset += width

    plt.legend(legend_handles, legend_labels)
    plt.ylim(0, max_count + 25)
    plt.xticks(inds + (width * num_series / 2),
               ['#%d' % (n + 1) for n in range(len(TEST_IMAGES))])
    plt.xlabel("Test images")
    plt.ylabel("Number of detections")
    plt.title("Improvements to Tag Detection")
    plt.show()

if __name__ == '__main__':
    main()
