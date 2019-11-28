Why do we have to wait after setting up the publisher?
https://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/
adding `latch=True` to the publisher constructor call does not really fix the
problem because only the last message will be sent, resulting in all of the
earlier messages being missed.
http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers

