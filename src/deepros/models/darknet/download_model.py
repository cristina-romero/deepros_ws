#!/usr/bin/env python
import os
import sys
import time
import yaml
import urllib
import argparse

required_keys = ['name', 'weights_url', 'cfg_url', 'labels_url']


def reporthook(count, block_size, total_size):
    """
    From http://blog.moleculea.com/2012/10/04/urlretrieve-progres-indicator/
    """
    global start_time
    if count == 0:
        start_time = time.time()
        return
    duration = (time.time() - start_time) or 0.01
    progress_size = int(count * block_size)
    speed = int(progress_size / (1024 * duration))
    percent = int(count * block_size * 100 / total_size)
    sys.stdout.write("\r...%d%%, %d MB, %d KB/s, %d seconds passed" %
                    (percent, progress_size / (1024 * 1024), speed, duration))
    sys.stdout.flush()


def parse_readme_frontmatter(dirname):
    readme_filename = os.path.join(dirname, 'config.yaml')
    with open(readme_filename) as f:
        frontmatter = yaml.load(f)
    assert all(key in frontmatter for key in required_keys)
    return dirname, frontmatter


def valid_dirname(dirname):
    try:
        return parse_readme_frontmatter(dirname)
    except Exception as e:
        print('ERROR: {}'.format(e))
        raise argparse.ArgumentTypeError(
            'Must be valid model directory with a correct config.yaml')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Download trained model binary.')
    parser.add_argument('dirname', type=valid_dirname)
    args = parser.parse_args()

    # A tiny hack: the dirname validator also returns readme YAML frontmatter.
    dirname = args.dirname[0]
    frontmatter = args.dirname[1]
    weights_filename = os.path.join(dirname, frontmatter['name'] + '.weights')
    cfg_filename = os.path.join(dirname, frontmatter['name'] + '.cfg')
    labels_filename = os.path.join(dirname, frontmatter['name'] + '.labels')

    # Check if model exists.
    if os.path.exists(weights_filename) and os.path.exists(cfg_filename) and os.path.exists(labels_filename):
        print("Model already exists.")
        sys.exit(0)

    # Download model.
    urllib.urlretrieve(
        frontmatter['weights_url'], weights_filename, reporthook)
        
    urllib.urlretrieve(
        frontmatter['cfg_url'], cfg_filename, reporthook)
        
    urllib.urlretrieve(
        frontmatter['labels_url'], labels_filename, reporthook)
