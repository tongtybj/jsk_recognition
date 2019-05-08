#!/usr/bin/env python

import argparse
import multiprocessing

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
        target=jsk_data.download_data,
        args=args,
        kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'jsk_perception'

    download_data(
        pkg_name=PKG,
        path='learning_datasets/kitchen_dataset.tgz',
        url='https://drive.google.com/uc?id=1iBSxX7I0nFDJfYNpFEb1caSQ0nl4EVUa',
        md5='61d6fcda7631fd0a940bc751c8f21031',
        quiet=quiet,
        extract=True,
    )

    download_data(
        pkg_name=PKG,
        path='learning_datasets/2019-04-28-07-04-27_mask_rcnn_drone_tracking_training_data_voc.tar.gz',
        url='https://drive.google.com/uc?id=1j4rMaKmbA8tPV9Yr9rIYiFfdjgtcb-Mp',
        md5='d3b5cb51061947994792978008621499',
        quiet=quiet,
        extract=True,
    )

    download_data(
        pkg_name=PKG,
        path='learning_datasets/2019-04-28-07-04-27_ssd_drone_tracking_training_data.tar.gz',
        url='https://drive.google.com/uc?id=1sUvHLHS8USPZEh40zzDtbUAW2xDWsTYL',
        md5='29155632d53346af67e08fff9bcebcc8',
        quiet=quiet,
        extract=True,
    )

if __name__ == '__main__':
    main()
