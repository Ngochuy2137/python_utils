import rospy
from .printer import Printer
global_printer = Printer()
def shutdown_node(msg=''):
    global_printer.print_red(f"Shutting down the node: {msg}")
    rospy.signal_shutdown("User requested shutdown")