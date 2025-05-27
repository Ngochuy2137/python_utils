import rospy
from .printer import Printer
from visualization_msgs.msg import Marker

global_printer = Printer()
def shutdown_node(msg=''):
    global_printer.print_red(f"Shutting down the node: {msg}")
    rospy.signal_shutdown("User requested shutdown")

def publish_text_marker(text, position, topic_name, marker_id=0, frame_id="map", color=(1.0, 1.0, 1.0), scale=0.3):
    """
    Gửi một TEXT marker đến RViz tại vị trí cụ thể

    Args:
        text (str): Nội dung chữ cần hiển thị
        position (tuple): Tọa độ (x, y, z) hiển thị chữ
        marker_id (int): ID marker, mỗi marker nên có id khác nhau nếu hiển thị nhiều cái cùng lúc
        frame_id (str): Khung tọa độ (ví dụ "map", "base_link")
        color (tuple): Màu chữ dạng (r, g, b), giá trị từ 0.0 đến 1.0
        scale (float): Kích thước chữ
    """
    print('pub text')
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "text_marker"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.w = 1.0  # giữ nguyên hướng chữ

    marker.scale.z = scale
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    marker.text = text
    publisher = rospy.Publisher(topic_name, Marker, queue_size=10)
    for _ in range(100):
        publisher.publish(marker)
        rospy.sleep(0.01)

# main
if __name__ == "__main__":
    rospy.init_node('ros_node_handle', anonymous=True)
    try:
        publish_text_marker("Hello ROS!", (0, 0, 1))
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown_node("ROS node interrupted")