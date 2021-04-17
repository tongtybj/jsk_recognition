#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_perception'

    download_data(
        pkg_name=PKG,
        path='test_data/2016-04-05-17-19-43_draw_bbox.tgz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vTDVTTmtSUXRPRlU',
        md5='9516566c66391835f85ac9cc44cbfc4b',
        extract=True,
        compressed_bags=['test_data/2016-04-05-17-19-43_draw_bbox/vision.bag'],
    )

    download_data(
        pkg_name=PKG,
        path='test_data/2016-04-06-08-16-08_img_cpi_decomposer.tgz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vcjlrOUN5LWM3dXM',
        md5='605a4e74f8dbdbdf33f66a48e713cf6d',
        extract=True,
        compressed_bags=[
            'test_data/2016-04-06-08-16-08_img_cpi_decomposer/vision.bag',
        ],
    )

    ### OTB100 KiteSurf for visual tracking 
    download_data(
        pkg_name=PKG,
        path='test_data/KiteSurf.zip',
        url='https://drive.google.com/uc?id=1mXxVnjQfTLNMCm-gTibvl-Y27q8sA0aO',
        md5='035b0fdf30357482edf1523e126da538',
        extract=True,
    )

    ### OTB100 Biker  for visual tracking 
    download_data(
        pkg_name=PKG,
        path='test_data/Biker.zip',
        url='https://drive.google.com/uc?id=1qIVvFDDGaGL2VFTJmBnYkbYD0twMksdV',
        md5='0c05d3d3ba6d450e8c7a8a9b304f86af',
        extract=True,
    )

if __name__ == '__main__':
    main()
