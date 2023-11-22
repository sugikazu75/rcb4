import os
import os.path as osp

import gdown


data_dir = osp.abspath(osp.dirname(__file__))
_default_cache_dir = osp.expanduser('~/.rcb4')


def get_cache_dir():
    return os.environ.get('RCB4_CACHE_DIR', _default_cache_dir)


def kondoh7_elf():
    target_path = osp.join(get_cache_dir(), 'elf',
                           '2023-11-23-kondoh7.elf')
    gdown.cached_download(
        url='https://drive.google.com/uc?id=1tv4spo4lnumkHeR43-6CGfL13ZZH_i7X',  # NOQA
        path=target_path,
        md5='22c90e8f3f2230da7ac0e0895429b9c5',
        quiet=True,
    )
    return target_path
