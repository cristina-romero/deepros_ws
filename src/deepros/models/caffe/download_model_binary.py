#!/usr/bin/env python
import os
import sys
import time
import yaml
import hashlib
import argparse
import tarfile
import re

from six.moves import urllib

required_keys = ['caffemodel', 'caffemodel_url', 'sha1', 'source']


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
    readme_filename = os.path.join(dirname, 'readme.md')
    with open(readme_filename) as f:
        lines = [line.strip() for line in f.readlines()]
    top = lines.index('---')
    bottom = lines.index('---', top + 1)
    frontmatter = yaml.load('\n'.join(lines[top + 1:bottom]))
    assert all(key in frontmatter for key in required_keys)
    return dirname, frontmatter


def valid_dirname(dirname):
    try:
        return parse_readme_frontmatter(dirname)
    except Exception as e:
        print('ERROR: {}'.format(e))
        raise argparse.ArgumentTypeError(
            'Must be valid Caffe model directory with a correct readme.md')

# Closure-d function for checking SHA1.
def model_checks_out(filename, sha1):
    with open(filename, 'rb') as f:
        return hashlib.sha1(f.read()).hexdigest() == sha1

def download_caffe_zoo_model(caffemodel_url, model_filename, model_sha1):
    urllib.request.urlretrieve(caffemodel_url, model_filename, reporthook)
    if not model_checks_out(filename=model_filename, sha1=model_sha1):
        print('ERROR: model did not download correctly! Run this again.')
        sys.exit(1)
    
def download_places_model(caffemodel_url, model_filename, model_sha1):
    tarfname = os.path.basename(caffemodel_url)
    urllib.request.urlretrieve(caffemodel_url, tarfname, reporthook)
    if not model_checks_out(filename=tarfname, sha1=model_sha1):
        print('ERROR: model did not download correctly! Run this again.')
        sys.exit(1)
      
    extract_dir = 'places_extract_model'
    if (tarfname.endswith('tar.gz')):
        tar = tarfile.open(tarfname, 'r:gz')
        tar.extractall(path=extract_dir)
        tar.close()
    
        for (dirpath, dirnames, filenames) in os.walk(extract_dir, topdown=False):        
            for name in filenames:
                if re.match('.*deploy.*.pro(to)?txt$', name):
                    os.rename(os.path.join(dirpath, name), os.path.join(dirname, 'deploy.prototxt'))
                elif name.endswith('.caffemodel'):
                    os.rename(os.path.join(dirpath, name), model_filename)
                else:
                    os.remove(os.path.join(dirpath, name))
                  
            for name in dirnames:
                os.rmdir(os.path.join(dirpath, name))
        os.rmdir(extract_dir)
    os.remove(tarfname)
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Download trained model binary.')
    parser.add_argument('dirname', type=valid_dirname)
    args = parser.parse_args()

    # A tiny hack: the dirname validator also returns readme YAML frontmatter.
    dirname = args.dirname[0]
    frontmatter = args.dirname[1]
    model_filename = os.path.join(dirname, frontmatter['caffemodel'])

    # Check if model exists.
    if os.path.exists(model_filename) and model_checks_out():
        print("Model already exists.")
        sys.exit(0)

    # Download and verify model.
    if frontmatter['source'] == 'caffe_zoo':
        download_caffe_zoo_model(frontmatter['caffemodel_url'], model_filename, frontmatter['sha1'])
    elif frontmatter['source'] == 'places':
        download_places_model(frontmatter['caffemodel_url'], model_filename, frontmatter['sha1'])
    else:
        print("ERROR: source not known.")
        sys.exit(1)
