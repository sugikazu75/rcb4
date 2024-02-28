#!/usr/bin/env python

import os.path as osp

import gdown
import rosgraph
import rospkg
import rospy
import tf
from urdf_parser_py.urdf import URDF


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

    if rospy.get_param('~connect_tf', False):
        robot = URDF.from_parameter_server(
            key=clean_namespace + '/robot_description')
        root_link = robot.get_root()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            br = tf.TransformBroadcaster()
            br.sendTransform((0, 0, 0),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             root_link,
                             "map")
