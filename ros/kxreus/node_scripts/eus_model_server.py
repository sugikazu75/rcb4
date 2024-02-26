#!/usr/bin/env python

import hashlib
import os
import tempfile

from filelock import FileLock
import rospkg
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from urdfeus.urdf2eus import urdf2eus


def checksum_md5(filename, blocksize=8192):
    """Calculate md5sum.

    Parameters
    ----------
    filename : str or pathlib.Path
        input filename.
    blocksize : int
        MD5 has 128-byte digest blocks (default: 8192 is 128x64).

    Returns
    -------
    md5 : str
        calculated md5sum.
    """
    filename = str(filename)
    hash_factory = hashlib.md5()
    with open(filename, 'rb') as f:
        for chunk in iter(lambda: f.read(blocksize), b''):
            hash_factory.update(chunk)
    return hash_factory.hexdigest()


class EusModelServer(object):

    def __init__(self):
        full_namespace = rospy.get_namespace()
        last_slash_pos = full_namespace.rfind('/')
        self.clean_namespace = full_namespace[:last_slash_pos] \
            if last_slash_pos != 0 else ''

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            urdf = rospy.get_param(
                self.clean_namespace + '/robot_description',
                None)
            if urdf is not None:
                tmp_file = tempfile.mktemp()
                with open(tmp_file, "w") as f:
                    f.write(urdf)
                md5sum = checksum_md5(tmp_file)

                r = RobotModel()
                with open(tmp_file, 'r') as f:
                    with no_mesh_load_mode():
                        r.load_urdf_file(f)
                robot_name = r.urdf_robot_model.name

                rospack = rospkg.RosPack()
                kxreus_path = rospack.get_path('kxreus')
                eus_path = os.path.join(
                    kxreus_path, 'models', '{}.l'.format(md5sum))
                if os.path.exists(eus_path):
                    rospy.set_param(self.clean_namespace + '/eus_robot_name',
                                    robot_name)
                    rospy.set_param(self.clean_namespace + '/eusmodel_hash',
                                    md5sum)
                    continue

                lock_path = eus_path + ".lock"
                lock = FileLock(lock_path, timeout=10)
                try:
                    with lock:
                        with open(eus_path, 'w') as f:
                            urdf2eus(tmp_file, fp=f)
                        rospy.loginfo(
                            'Eusmodel is saved to {}'.format(eus_path))
                    os.remove(lock_path)
                finally:
                    if os.path.exists(tmp_file):
                        os.remove(tmp_file)


if __name__ == '__main__':
    rospy.init_node('model_server')
    server = EusModelServer()
    server.run()
