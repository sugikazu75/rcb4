import os.path as osp

import gdown
from kxr_models.ros import get_namespace
import rospkg
import rospy


def download_urdf_mesh_files(namespace=None):
    if namespace is None:
        namespace = get_namespace()
    port = rospy.get_param(namespace + '/model_server_port', 8123)
    server_ip = rospy.get_param(namespace + '/model_server_ip',
                                'localhost')

    urdf_hash = rospy.get_param(namespace + '/urdf_hash', None)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown() and urdf_hash is None:
        rate.sleep()
        rospy.loginfo('Waiting rosparam {} set'.format(
            namespace + '/urdf_hash'))
        urdf_hash = rospy.get_param(namespace + '/urdf_hash', None)

    server_url = "http://{}:{}/urdf/{}.tar.gz".format(
        server_ip, port, urdf_hash)

    rospack = rospkg.RosPack()
    kxr_models_path = rospack.get_path('kxr_models')

    compressed_urdf_path = osp.join(kxr_models_path, 'models', 'urdf',
                                    "{}.tar.gz".format(urdf_hash))
    while not osp.exists(compressed_urdf_path):
        rospy.loginfo('Waiting {} from server'.format(compressed_urdf_path))
        rospy.sleep(1.0)
        gdown.cached_download(
            url=server_url,
            path=compressed_urdf_path)
    gdown.extractall(osp.join(kxr_models_path, 'models', 'urdf',
                              "{}.tar.gz".format(urdf_hash)))
