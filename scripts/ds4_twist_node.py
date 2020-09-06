#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from ds4_driver.msg import Status


class StatusToTwist(object):
    def __init__(self):
        self._topic = rospy.get_param('~topic', 'cmd_vel')
        self._rate = rospy.get_param('~rate', 10.0)
        self._stamped = rospy.get_param('~stamped', False)
        if self._stamped:
            self._cls = TwistStamped
            self._frame_id = rospy.get_param('~frame_id', 'base_link')
        else:
            self._cls = Twist
            
        self._to_pub = self._cls()
        self._inputs = rospy.get_param('~inputs')
        self._scales = rospy.get_param('~scales')

        self._attrs = []
        for attr in Status.__slots__:
            if attr.startswith('axis_') or attr.startswith('button_'):
                self._attrs.append(attr)

        self._pub = rospy.Publisher(self._topic, self._cls, queue_size=1)
        rospy.Subscriber('status', Status, self.cb_status, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/self._rate), self.cb_publish)

    def cb_status(self, msg):
        """
        :param msg:
        :type msg: Status
        :return:
        """
        input_vals = {}
        for attr in self._attrs:
            input_vals[attr] = getattr(msg, attr)

        # to_pub = self._cls()
        if self._stamped:
            self._to_pub.header.stamp = rospy.Time.now()
            self._to_pub.header.frame_id = self._frame_id
            twist = self._to_pub.twist
        else:
            twist = self._to_pub

        for vel_type in self._inputs:
            vel_vec = getattr(twist, vel_type)
            for k, expr in self._inputs[vel_type].items():
                scale = self._scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

    def cb_publish(self, _):
        self._pub.publish(self._to_pub)


def main():
    rospy.init_node('ds4_twist')

    StatusToTwist()

    rospy.spin()


if __name__ == '__main__':
    main()
