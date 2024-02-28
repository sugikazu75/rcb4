#!/usr/bin/env python

import os.path as osp

import rospy
import rosgraph
import rospkg
import gdown


if __name__ == '__main__':
    rospy.init_node('get_urdf_model')
    full_namespace = rospy.get_namespace()
    last_slash_pos = full_namespace.rfind('/')
    clean_namespace = full_namespace[:last_slash_pos] \
        if last_slash_pos != 0 else ''
    port = rospy.get_param(clean_namespace + '/model_server_port', 8123)
    urdf_hash = rospy.get_param(clean_namespace + '/urdf_hash')

    master = rosgraph.Master("")
    host = master.lookupNode("/rosout").split(':')[1][2:]
    server_url = "http://{}:{}/urdf/{}.tar.gz".format(
        host, port, urdf_hash)

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
