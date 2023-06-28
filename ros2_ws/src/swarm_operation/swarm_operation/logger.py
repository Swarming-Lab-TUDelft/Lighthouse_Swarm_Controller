from std_msgs.msg import String

class Logger():
    """
    Logger class for simultaneous logging to ROS terminal and GUI.
    """
    def __init__(self, ros_logger, msg_pub, mode="info"):
        self.ros_logger = ros_logger
        self.msg_pub = msg_pub
        self.mode = mode
    
    def info(self, msg):
        self.ros_logger.info(f"{msg}")
        self.msg_pub.publish(String(data=f"[info] {msg}"))
    
    def debug(self, msg):
        if self.mode == "debug":
            self.ros_logger.debug(f"{msg}")
            self.msg_pub.publish(String(data=f"[debug] {msg}"))