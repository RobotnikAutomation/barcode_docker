#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from barcode_docker import BarcodeDocker


def main():

    rospy.init_node("barcode_docker_node")

    rc_node = BarcodeDocker()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
