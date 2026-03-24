#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading

import rospy
from std_msgs.msg import Int8MultiArray

COLS = 10
ROWS = 50
GAP = " " * 10


def _normalize_ns(ns):
    ns = (ns or "").strip()
    return ns.strip("/")


def _topic(ns, tactile_name):
    """ ns=left_gripper, tactile_name=left -> /left_gripper/tactile/left"""
    n = _normalize_ns(ns)
    return "/{}/tactile/{}".format(n, tactile_name)


class TactileDualPrinter:
    def __init__(self):
        gripper_ns = rospy.get_param("~gripper_ns", "left_gripper")
        hz = float(rospy.get_param("~print_hz", 30.0))
        self._topic_left = _topic(gripper_ns, "left")
        self._topic_right = _topic(gripper_ns, "right")

        self._lock = threading.Lock()
        self._data_left = None
        self._data_right = None

        rospy.loginfo("gripper_ns=%s", gripper_ns)
        rospy.loginfo("subscribe tactile/left:  %s", self._topic_left)
        rospy.loginfo("subscribe tactile/right: %s", self._topic_right)

        rospy.Subscriber(self._topic_left, Int8MultiArray, self._cb_left, queue_size=1)
        rospy.Subscriber(self._topic_right, Int8MultiArray, self._cb_right, queue_size=1)

        period = max(0.02, 1.0 / hz) if hz > 0 else 0.05
        rospy.Timer(rospy.Duration(period), self._on_timer)

    def _cb_left(self, msg):
        with self._lock:
            self._data_left = list(msg.data)

    def _cb_right(self, msg):
        with self._lock:
            self._data_right = list(msg.data)

    def _on_timer(self, _evt):
        with self._lock:
            if self._data_left is None or self._data_right is None:
                return
            L, R = list(self._data_left), list(self._data_right)
        n = min(len(L), len(R), ROWS * COLS)
        if n < ROWS * COLS:
            rospy.logwarn_throttle(
                5.0,
                "tactile length is less than 500: left=%d right=%d, will only print the first %d numbers",
                len(L),
                len(R),
                n,
            )
        for row in range(ROWS):
            i0 = row * COLS
            if i0 + COLS > n:
                break
            left_seg = L[i0 : i0 + COLS]
            right_seg = R[i0 : i0 + COLS]
            left_s = " ".join("{:3d}".format(int(x)) for x in left_seg)
            right_s = " ".join("{:3d}".format(int(x)) for x in right_seg)
            print(left_s + GAP + right_s)
        print("", flush=True)


def main():
    rospy.init_node("tactile_dual_print", anonymous=True)
    TactileDualPrinter()
    rospy.spin()


if __name__ == "__main__":
    main()
