#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class VisualizationMarkerPublisher(Node):
    def __init__(self):
        super().__init__('visualization_marker_publisher') #node name ros2 uses
        

        # this is how you create a publisher!
        # publisher command is different for a marker!
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)
            # in example code, this is above the timer

        # need a timer for this publisher:
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.publish_marker)

        

    # callback function for publisher
    def publish_marker(self):
        """
        Callback function for publisher, defines details of marker
        
        p good explanation of this code: https://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
        """
        marker = Marker() # creates a variable "marker" with type Marker
        marker.header.frame_id = "base_link"; # puts marker in odom frame
        marker.header.stamp = self.get_clock().now().to_msg() # sets timestamp
       
        # these things identify this marker and will be overriden each time
        # this loop runs, but will not be overriden by markers in a diff frame
        # being published to the same topic
        marker.ns = "my_namespace"; # new marker with same namespace will override
        marker.id = 0 # new marker with same id will override

        marker.type = Marker.SPHERE; # could set to diff shape, like CUBE
        marker.action = Marker.ADD; # could also be DELETE

        marker.pose.position.x = 1.0 # relative to frame and time in header
        marker.pose.position.y = 2.0 # units in meters
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0 # units in meters
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0; # Don't forget to set the alpha! (determines opacity)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.vis_pub.publish( marker );


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node() # would be done automatically but better explicit
    rclpy.shutdown()

if __name__ == '__main__':
    main()