from mpl_toolkits.mplot3d import Axes3D
# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import random

class Plotter:
    def __init__(self, topic_name='plotter/visualization_marker'):
        self.colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'orange', 'purple', 'pink', 'brown', 'gray']
        self.simbol = ['o', '+', 'x', 's', 'p', 'h', 'd', 'v', '^', '<', '>', '1', '2', '3', '4', '8']
        self.colors_rviz = [
            ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red
            ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green
            ColorRGBA(1.0, 1.0, 0.0, 1.0),  # Yellow
            ColorRGBA(1.0, 0.0, 1.0, 1.0),  # Magenta
            ColorRGBA(0.0, 1.0, 1.0, 1.0),  # Cyan
            ColorRGBA(0.5, 0.5, 0.5, 1.0),  # Gray
            ColorRGBA(1.0, 0.5, 0.0, 1.0),  # Orange
            ColorRGBA(0.0, 0.0, 1.0, 1.0),  # Blue
            ColorRGBA(0.5, 0.0, 1.0, 1.0),  # Purple
            ColorRGBA(0.0, 0.5, 1.0, 1.0)   # Light Blue
        ]
        self.pub = rospy.Publisher(topic_name, MarkerArray, queue_size=1)
        self.last_marker_id = 0

    # def plot_samples(self, samples_simbols_list, title=''):
    #     # shuffle samples_simbols_list
    #     random.shuffle(samples_simbols_list)

    #     # check if each trajectory has 2 elements (trajectory and simbol) or not 
    #     data_with_simbol = True
    #     for i in range(len(samples_simbols_list)):
    #         if len(samples_simbols_list[i]) != 2:
    #             data_with_simbol = False
    #     if not data_with_simbol:
    #         # add default simbol for each trajectory
    #         samples_simbols_list = [[s, 'o'] for s in samples_simbols_list]

    #     figure = plt.figure()
    #     ax = figure.add_subplot(111, projection='3d')
    #     limit = min(len(self.colors), len(samples_simbols_list))
    #     for i in range(limit-1):
    #         samp_traj = samples_simbols_list[i][0]
    #         samp_simbol = samples_simbols_list[i][1]

    #         print('samp_traj: ', len(samp_traj))
    #         print('samp_simbol: ', samp_simbol)
    #         input()

    #         limit = len(self.colors)
    #         if i+1 >= limit:
    #             return
    #         ax.plot(samp_traj[:, 0], samp_traj[:, 1], samp_traj[:, 2], 
    #                     samp_simbol, color=self.colors[i], alpha=0.5, label='Test '+str(i+1) + 'Sample Trajectory')
            
    #         # Đặt chữ "end" tại điểm cuối của mỗi sample
    #         end_point = samp_traj[-1]  # Điểm cuối cùng của sample
    #         ax.text(end_point[0], end_point[1], end_point[2], 'end', color=self.colors[i], fontsize=10, ha='center')
        
    #     # Set limits
    #     x_min, x_max = 100000, -100000
    #     y_min, y_max = 100000, -100000
    #     z_min, z_max = 100000, -100000
    #     for samps_vs_simb in samples_simbols_list:
    #         samps = samps_vs_simb[0]
    #         for samp in samps:
    #             x_min = min(x_min, samp[0])
    #             x_max = max(x_max, samp[0])
    #             y_min = min(y_min, samp[1])
    #             y_max = max(y_max, samp[1])
    #             z_min = min(z_min, samp[2])
    #             z_max = max(z_max, samp[2])
        
    #     ax.set_xlim([x_min, x_max])
    #     ax.set_ylim([y_min, y_max])
    #     ax.set_zlim([z_min, z_max])
    #     ax.set_xlabel('X')
    #     ax.set_ylabel('Y')
    #     ax.set_zlabel('Z')

    #     plt.legend()
    #     plt.title('3D samples ' + title, fontsize=25)
    #     plt.show()

    def plot_samples(self, samples, title='', rotate_data_whose_y_up=False, plot_all=False):
        print('Plotting samples...')
        figure = plt.figure(num=1)
        ax = figure.add_subplot(111, projection='3d')

        x_min = 1000
        x_max = -1000
        y_min = 1000
        y_max = -1000
        z_min = 1000
        z_max = -1000
        for i in range (len(samples)):
            if i >= len(self.colors)-1:
                if not plot_all:
                    break
                color_current = self.colors[-1]
            else:
                color_current = self.colors[i]

            sample = np.array(samples[i])

            if rotate_data_whose_y_up:
                # change the order of the axis so that z is up and follow the right-hand rule
                x_data = sample[:, 0]
                y_data = -sample[:, 2]
                z_data = sample[:, 1]
            else:
                x_data = sample[:, 0]
                y_data = sample[:, 1]
                z_data = sample[:, 2]

            ax.plot(x_data, y_data, z_data, 
                        'o', color=color_current, alpha=0.5, label='Test '+str(i+1) + 'Sample Trajectory')
            x_min = min(x_min, x_data.min())
            x_max = max(x_max, x_data.max())
            y_min = min(y_min, y_data.min())
            y_max = max(y_max, y_data.max())
            z_min = min(z_min, z_data.min())
            z_max = max(z_max, z_data.max())
            
        ax.set_xlim([x_min, x_max])
        ax.set_ylim([y_min, y_max])
        ax.set_zlim([z_min, z_max])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.legend()
        plt.title('3D samples ' + title, fontsize=25)
        plt.show()
    
    def update_plot(self, one_trajectory_prediction, one_trajectory_label, color_idx, ax, swap_y_z=False):
        # Check if the color index is within the range
        if color_idx+1 >= len(self.colors):
            return

        # Determine axis order based on swap_y_z flag
        if swap_y_z:
            x_idx, y_idx, z_idx = 0, 2, 1  # Swap y and z
        else:
            x_idx, y_idx, z_idx = 0, 1, 2  # Default order

        # Plot actual and predicted trajectories
        ax.plot(one_trajectory_label[:, x_idx], one_trajectory_label[:, y_idx], one_trajectory_label[:, z_idx],
                    'o', color=self.colors[color_idx], alpha=0.5, label=f'Test {color_idx + 1} Actual Trajectory', markersize=10)
        ax.plot(one_trajectory_prediction[:, x_idx], one_trajectory_prediction[:, y_idx], one_trajectory_prediction[:, z_idx],
                    '+', color=self.colors[color_idx], label=f'Test {color_idx + 1} Predicted Trajectory', markersize=10)

        # Calculate limits
        all_data = np.concatenate((one_trajectory_label, one_trajectory_prediction), axis=0)
        ax.set_xlim([all_data[:, x_idx].min(), all_data[:, x_idx].max()])
        ax.set_ylim([all_data[:, y_idx].min(), all_data[:, y_idx].max()])
        ax.set_zlim([all_data[:, z_idx].min(), all_data[:, z_idx].max()])

        # Set labels based on axis order
        ax.set_xlabel('X')
        ax.set_ylabel('Z' if swap_y_z else 'Y')
        ax.set_zlabel('Y' if swap_y_z else 'Z')

        # Calculate and display the distance for the last points
        actual_point = one_trajectory_label[-1, [x_idx, y_idx, z_idx]]
        predicted_point = one_trajectory_prediction[-1, [x_idx, y_idx, z_idx]]
        distance = np.linalg.norm(actual_point - predicted_point)

        # Plot the distance between the last points with a line and annotate the value
        ax.plot([actual_point[0], predicted_point[0]],
                    [actual_point[1], predicted_point[1]],
                    [actual_point[2], predicted_point[2]],
                    linestyle='--', color='red', label=f'Error Distance: {distance:.2f}')
        
        # Annotate the distance value with a larger fontsize
        mid_point = (actual_point + predicted_point) / 2
        ax.text(mid_point[0], mid_point[1], mid_point[2], f'{distance:.2f}', color='red', fontsize=20)  # Change fontsize here

    def plot_predictions(self, predictions, test_labels, lim_plot_num=None, swap_y_z=False, title=''):
        if lim_plot_num:
            predictions = predictions[:lim_plot_num]
            test_labels = test_labels[:lim_plot_num]
        lim_plot_num = min(len(self.colors), len(predictions))
        figure = plt.figure()
        ax = figure.add_subplot(111, projection='3d')
        for i in range (lim_plot_num):
            self.update_plot(predictions[i], test_labels[i], i, ax, swap_y_z=False)

        plt.legend()
        plt.title('3D Predictions ' + title, fontsize=18)
        plt.show()

    def plot_samples_rviz(self, segments_plot, title, frame_id='world'):
        marker_array = MarkerArray()  # Tạo một MarkerArray để chứa tất cả các marker
        marker_id = 0  # ID của marker, sẽ tăng dần để đảm bảo mỗi marker là duy nhất
        color_index = 1  # Bỏ qua màu đỏ (0) để tránh trùng màu 'o'
        
        # Tạo marker cho tiêu đề
        title_marker = Marker()
        title_marker.header.frame_id = frame_id
        title_marker.header.stamp = rospy.Time.now()
        title_marker.ns = "segments"
        title_marker.id = marker_id
        marker_id += 1
        title_marker.type = Marker.TEXT_VIEW_FACING  # Dùng TEXT_VIEW_FACING để hiển thị văn bản
        title_marker.action = Marker.ADD
        title_marker.pose.position.x = -2  # Vị trí hiển thị tiêu đề
        title_marker.pose.position.y = -2
        title_marker.pose.position.z = 4  # Đặt tiêu đề cao hơn
        title_marker.pose.orientation.w = 1.0
        title_marker.scale.z = 0.5  # Kích thước của văn bản
        title_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # Màu trắng
        title_marker.text = title  # Nội dung của tiêu đề
        marker_array.markers.append(title_marker)  # Thêm tiêu đề vào MarkerArray

        # Publish từng segment
        object_color_index = 0
        for segment, symbol in segments_plot:
            # Tạo marker cho từng segment
            

            # Thiết lập loại marker dựa trên ký hiệu
            if symbol == 'o':
                # Tạo marker cho `SPHERE_LIST`
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = "segments"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.SPHERE_LIST
                marker.action = Marker.ADD
                marker.color = self.colors_rviz[0]
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.pose.orientation.w = 1.0
                object_color_index = 0
                for point_data in segment:
                    point = Point()
                    point.x = point_data[0]
                    point.y = point_data[1]
                    point.z = point_data[2]
                    marker.points.append(point)
                marker_array.markers.append(marker)

            elif symbol == 'x':
                # Tạo nhiều marker `TEXT_VIEW_FACING` cho từng điểm 'x'
                object_color_index = color_index % len(self.colors_rviz)
                for point_data in segment:
                    marker = Marker()
                    marker.header.frame_id = frame_id
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "segments"
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.action = Marker.ADD
                    marker.text = 'x'
                    marker.scale.z = 0.05
                    marker.color = self.colors_rviz[object_color_index]
                    marker.pose.position.x = point_data[0]
                    marker.pose.position.y = point_data[1]
                    marker.pose.position.z = point_data[2]
                    marker.pose.orientation.w = 1.0
                    marker_array.markers.append(marker)
                color_index += 1

            else:
                rospy.logwarn("Invalid symbol. Skipping segment.")
                continue

            # Tạo marker "end" tại điểm cuối của segment
            end_marker = Marker()
            end_marker.header.frame_id = frame_id
            end_marker.header.stamp = rospy.Time.now()
            end_marker.ns = "segments"
            end_marker.id = marker_id
            marker_id += 1
            end_marker.type = Marker.TEXT_VIEW_FACING
            end_marker.action = Marker.ADD
            end_marker.pose.position.x = segment[-1][0] + 0.05
            end_marker.pose.position.y = segment[-1][1] + 0.05
            end_marker.pose.position.z = segment[-1][2] + 0.05  # Hiển thị chữ "end" ngay trên điểm cuối
            end_marker.pose.orientation.w = 1.0
            end_marker.scale.z = 0.05  # Kích thước chữ "end"
            end_marker.color = self.colors_rviz[object_color_index]  # Màu đỏ cho chữ "end"
            end_marker.text = "end"  # Văn bản cho điểm cuối
            marker_array.markers.append(end_marker)  # Thêm end_marker vào MarkerArray
        self.last_marker_id = marker_id
        # Publish toàn bộ MarkerArray
        self.pub.publish(marker_array)
        rospy.loginfo("All segments, title, and end markers published to RViz.")