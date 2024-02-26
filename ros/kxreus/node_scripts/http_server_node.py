#!/usr/bin/env python

from http.server import HTTPServer
from http.server import SimpleHTTPRequestHandler
import os
import threading

from filelock import FileLock
from filelock import Timeout
import rospkg
import rospy


class CustomHTTPRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, directory=None, **kwargs):
        self.base_directory = directory
        super().__init__(*args, **kwargs)

    def translate_path(self, path):
        path = super().translate_path(path)
        rel_path = os.path.relpath(path, os.getcwd())
        full_path = os.path.join(self.base_directory, rel_path) \
            if self.base_directory else path
        return full_path

    def do_GET(self):
        filepath = self.translate_path(self.path)
        lock_path = filepath + ".lock"

        if os.path.exists(lock_path):
            try:
                with FileLock(lock_path, timeout=1):
                    super().do_GET()
            except Timeout:
                self.send_response(503)
                self.end_headers()
                self.wfile.write(
                    b"Service Unavailable: File is currently locked.")
        else:
            super().do_GET()


class ThreadedHTTPServer(object):
    def __init__(self, host, port, handler_class, directory):
        def handler_factory(*args, **kwargs):
            return handler_class(
                *args, directory=directory, **kwargs)
        self.server = HTTPServer((host, port), handler_factory)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True

    def start(self):
        self.server_thread.start()
        rospy.loginfo("HTTP Server Running...")

    def stop(self):
        self.server.shutdown()
        self.server.server_close()
        rospy.loginfo("HTTP Server Stopped.")


if __name__ == '__main__':
    rospy.init_node('http_server_node')

    rospack = rospkg.RosPack()
    kxreus_path = rospack.get_path('kxreus')
    www_directory = os.path.join(kxreus_path, 'models')

    full_namespace = rospy.get_namespace()
    last_slash_pos = full_namespace.rfind('/')
    clean_namespace = full_namespace[:last_slash_pos] \
        if last_slash_pos != 0 else ''

    port = rospy.get_param(clean_namespace + '/port', 8000)
    server = ThreadedHTTPServer(
        '0.0.0.0', port, CustomHTTPRequestHandler, www_directory)
    server.start()

    rospy.spin()

    server.stop()
