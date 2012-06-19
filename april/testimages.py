#!/usr/bin/env python

from matplotlib import pyplot as plt
import subprocess
import sys

TAGTEST_BIN = "build/tagtest"

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
    a_values = range(0, 100, 1)
    ideal_counts = get_image_counts(['-rx'])
    test_counts = [get_image_counts(['-rx', '-D', '-a %d' % a])
                   for a in a_values]
    all_data = zip(ideal_counts, zip(*test_counts))
    for data_series in reversed(sorted(all_data)):
        ideal = data_series[0]
        actuals = data_series[1]
        plt.plot(a_values, [ideal] * len(a_values), 'k:')
        plt.plot(a_values, actuals, label='%d' % ideal)
    plt.ylim(0, max(ideal_counts) + 5)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
