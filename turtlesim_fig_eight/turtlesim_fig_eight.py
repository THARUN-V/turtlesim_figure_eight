import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose

import time
import math

class TurtlesimFigEight(Node):

    def __init__(self):
        super().__init__('TurtleSimFigEight')

        self.turtle2_name = "foxy"

        ## flag to get turtle1 info
        self.get_t1_x_y = False
        self.t1_x = 0
        self.t1_y = 0
        self.spawned_t2 = False

        # flag to check if turtle1 pose is available
        self.turtle1_pose_flag = False
        # flag to localize turtle1 and turtle2
        self.localize_turtle1_flag = False
        self.localize_turtle2_flag = False

        # variable to keep track of initial position of turtle1 and turtle2
        self.initial_x_t1 = None
        self.initial_y_t1 = None
        self.initial_x_t2 = None
        self.initial_y_t2 = None

        #flag to stop turtle1 and turtle2
        self.stop_t1_flag = False
        self.stop_t2_flag = False

        # variable to update the change in pose of turtle1 after localizing
        self.pub_count_t1 = 0
        self.pub_count_t2 = 0

        #flag to start turtle2
        self.start_turtle2 = False

        ### publisher and subscriber for turtle1
        self.turtle1_pose_sub = self.create_subscription(Pose,"/turtle1/pose",self.turtle1_pose_cb,10)
        self.turtle1_cmd_vel_pub = self.create_publisher(Twist,"/turtle1/cmd_vel",10)

        ### publisher and subscriber for turtle2
        self.turtle2_pose_sub = self.create_subscription(Pose,"/"+self.turtle2_name+"/pose",self.turtle2_pose_cb,10)
        self.turtle2_cmd_vel_pub = self.create_publisher(Twist,"/"+self.turtle2_name+"/cmd_vel",10)
        
        
        self.turtle_spawn_srv = self.create_client(Spawn,"spawn")
        while not self.turtle_spawn_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting for service ....")
        self.req = Spawn.Request()        


    def turtle1_pose_cb(self,msg):
        
        # check for turtle1 pose and set turtle2 pose
        if not self.turtle1_pose_flag:
            # self.get_logger().info("-----------------")
            # self.spawn_request(msg.x,msg.y,self.turtle2_name)
            self.t1_x = msg.x
            self.t1_y = msg.y
            self.get_t1_x_y = True
            self.turtle1_pose_flag = True
        
        # localize tutle1 to check if it completed one full circle
        if self.turtle1_pose_flag and not self.localize_turtle1_flag:

            self.initial_x_t1 = float(str(msg.x)[:4])

            # self.initial_x_t1 = msg.x
            self.initial_y_t1 = float(str(msg.y)[:4])

            t1_localize_msg = Twist()
            t1_localize_msg.linear.x = 0.1
            self.turtle1_cmd_vel_pub.publish(t1_localize_msg)
            self.get_logger().info("--------LOCALIZED T1---------")
            self.localize_turtle1_flag = True

        # make a half circle using turtle1
        # first check if turtle2 is spawned and turtle1 is localized
        if self.initial_x_t1 == float(str(msg.x)[:4]) and self.initial_y_t1 == float(str(msg.y)[:4]) and not \
            self.stop_t1_flag and self.turtle1_pose_flag and \
            self.localize_turtle1_flag and self.pub_count_t1>65:

            self.get_logger().info("start x : {} , current x : {}".format(self.initial_x_t1,float(str(msg.x)[:4])))
            self.get_logger().info("--------------- STOP TURTLE1--------------")
            self.stop_t1_flag = True
            self.start_turtle2 = True
            
        # publish until it makes full circle
        if not self.stop_t1_flag:
            twist_msg = Twist()
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.1
            self.turtle1_cmd_vel_pub.publish(twist_msg)

        ### check for turtle1 completing circle and spawn turtle2
        if self.stop_t1_flag and not self.spawned_t2:
            self.spawn_request(self.t1_x,self.t1_y,self.turtle2_name)
            self.spawned_t2 = True


        self.pub_count_t1 += 1

    def turtle2_pose_cb(self,msg):

        ## check if turtle1 has completed drawing upper circle
        if self.start_turtle2:
            # localize turtle2
            if not self.localize_turtle2_flag:

                self.initial_x_t2 = float(str(msg.x)[:4])

                # self.initial_x_t1 = msg.x
                self.initial_y_t2 = float(str(msg.y)[:4])

                t2_localize_msg = Twist()
                t2_localize_msg.linear.x = 0.1
                self.turtle2_cmd_vel_pub.publish(t2_localize_msg)
                self.get_logger().info("--------LOCALIZED T2---------")
                self.localize_turtle2_flag = True
            
            # make a lower circle using turtle2
            if not self.stop_t2_flag:
                twist_msg = Twist()
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = -0.1
                self.turtle2_cmd_vel_pub.publish(twist_msg) 

            #check if turtle2 completer circle
            if self.initial_x_t2 == float(str(msg.x)[:4]) and self.initial_y_t2 == float(str(msg.y)[:4]) and self.localize_turtle2_flag and self.pub_count_t2>65:
                self.stop_t2_flag = True
                self.get_logger().info("--------------- STOP TURTLE2--------------")

            self.pub_count_t2 += 1
        
        
    
    def twist_callback(self):
        msg = Twist()
        msg.linear.x = 0.25
        msg.angular.z = 0.25

        self.cmd_vel_pub.publish(msg)

    # service call back to spawn new turtle in turtlesim
    def spawn_request(self,x,y,name):
        
        self.req.x = x
        self.req.y = y
        self.req.name = name
        self.future = self.turtle_spawn_srv.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    turtle_sim_fig_eight = TurtlesimFigEight()

    rclpy.spin(turtle_sim_fig_eight)

    turtle_sim_fig_eight.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()