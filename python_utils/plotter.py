from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import random
from plotly.graph_objects import Figure, Scatter3d
from plotly.offline import plot
import os
import plotly.graph_objects as go

from .subutils.prediction_plotter import PredictionPlotter
class Plotter:
    def __init__(self, ):
        self.colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'orange', 'purple', 'pink', 'brown', 'gray']
        self.simbol = ['o', '+', 'x', 's', 'p', 'h', 'd', 'v', '^', '<', '>', '1', '2', '3', '4', '8']
        self.last_marker_id = 0
        self.plotter_plotply = PredictionPlotter()

    def plot_predictions_plotly(self, inputs, labels, predictions, title='', rotate_data_whose_y_up=False, save_plot=False, font_size_note=12, show_all_as_default=True):
        self.plotter_plotply.plot_predictions(inputs=inputs, labels=labels, predictions=predictions, 
                                                rotate_data_whose_y_up=rotate_data_whose_y_up, 
                                                title=title,
                                                save_plot=save_plot, font_size_note=font_size_note,
                                                show_all_as_default=show_all_as_default)
        
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

    def plot_trajectory_dataset_matplotlib(
        self,
        samples,
        title='',
        rotate_data_whose_y_up=False,
        plot_all=False,
        shuffle=False,
        plot_xy_only=False,      # <-- thêm tham số mới
        label_list=None,
        label_font_size=20
    ):
        # Tạo figure và axes tuỳ theo chế độ 2D hay 3D
        if plot_xy_only:
            fig, ax = plt.subplots(figsize=(12, 12))   # 2D
        else:
            fig = plt.figure(num=1, figsize=(12, 12))
            ax = fig.add_subplot(111, projection='3d')  # 3D

        if shuffle:
            random.shuffle(samples)

        # Khởi tạo biến min/max cho từng chiều (vẫn dùng chung)
        x_min = y_min = z_min = float('inf')
        x_max = y_max = z_max = float('-inf')

        for i, sample in enumerate(samples):
            if i >= len(self.colors) - 1 and not plot_all:
                break
            color_current = self.colors[i] if i < len(self.colors) - 1 else self.colors[-1]

            sample = np.array(sample)
            # Chuyển trục nếu cần
            if rotate_data_whose_y_up:
                x_data = sample[:, 0]
                y_data = -sample[:, 2]
                z_data = sample[:, 1]
            else:
                x_data = sample[:, 0]
                y_data = sample[:, 1]
                z_data = sample[:, 2]

            # Vẽ
            if label_list is not None and i < len(label_list):
                label = label_list[i]
            else:
                label = f'Test {i+1} Sample Trajectory'
            if plot_xy_only:
                ax.plot(x_data, y_data, 'o',
                        color=color_current, alpha=0.5,
                        label=label)
                ax.text(x_data[-1], y_data[-1], 'end',
                        color=color_current, fontsize=10)
            else:
                ax.plot(x_data, y_data, z_data, 'o',
                        color=color_current, alpha=0.5,
                        label=label)
                ax.text(x_data[-1], y_data[-1], z_data[-1], 'end',
                        color=color_current, fontsize=10)

            # Cập nhật min/max
            x_min, x_max = min(x_min, x_data.min()), max(x_max, x_data.max())
            y_min, y_max = min(y_min, y_data.min()), max(y_max, y_data.max())
            z_min, z_max = min(z_min, z_data.min()), max(z_max, z_data.max())

        # Tính phạm vi và tâm
        x_range, y_range, z_range = x_max-x_min, y_max-y_min, z_max-z_min
        max_range = max(x_range, y_range, z_range) + 0.5
        x_mid, y_mid, z_mid = (x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2

        # Set giới hạn trục
        ax.set_xlim(x_mid - max_range/2, x_mid + max_range/2)
        ax.set_ylim(y_mid - max_range/2, y_mid + max_range/2)
        if not plot_xy_only:
            ax.set_zlim(z_mid - max_range/2, z_mid + max_range/2)

        # Gán nhãn
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        if not plot_xy_only:
            ax.set_zlabel('Z')
        ax.legend(fontsize=label_font_size)
        

        # plt.legend()
        # Điều chỉnh tiêu đề cho 2D hay 3D
        kind = '2D' if plot_xy_only else '3D'
        plt.title(f'{kind} samples {title}', fontsize=25)
        plt.show()
    
    def plot_trajectory_dataset_plotly(self, trajectories, title='', rotate_data_whose_y_up=False, save_plot=False):
        fig = Figure()

        # Tạo một danh sách các màu sắc
        colors = [
            f'rgb({random.randint(0,255)}, {random.randint(0,255)}, {random.randint(0,255)})'
            for _ in range(len(trajectories))
        ]

        for i, traj in enumerate(trajectories):
            # Xử lý dữ liệu nếu cần xoay trục
            if rotate_data_whose_y_up:
                x_data = traj[:, 0]
                y_data = -traj[:, 2]  # Đảo chiều trục Y
                z_data = traj[:, 1]
            else:
                x_data = traj[:, 0]
                y_data = traj[:, 1]
                z_data = traj[:, 2]

            # Thêm quỹ đạo vào biểu đồ
            fig.add_trace(Scatter3d(
                x=x_data,
                y=y_data,
                z=z_data,
                mode='lines+markers',  # Hiển thị cả đường nối và điểm
                marker=dict(
                    size=4,
                    color=colors[i],  # Mỗi quỹ đạo có màu riêng
                    opacity=0.8
                ),
                line=dict(
                    color=colors[i],  # Màu của đường nối
                    width=2
                ),
                name=f'Trajectory {i}'  # Đặt tên cho quỹ đạo
            ))

            # Thêm chữ "end" bên cạnh điểm cuối
            fig.add_trace(Scatter3d(
                x=[x_data[-1]],
                y=[y_data[-1]],
                z=[z_data[-1]],
                mode='text',
                text=["end"],
                textposition="top right",
                textfont=dict(size=10, color=colors[i]),
                showlegend=False  # Không hiển thị trong chú giải
            ))

        # Cập nhật bố cục
        fig.update_layout(
            title=dict(
                text=f"{title} - Trajectories",  # Nội dung tiêu đề
                font=dict(size=32),  # Kích thước chữ
                x=0.5  # Canh giữa tiêu đề
            ),

            legend=dict(
                font=dict(size=16),  # Kích thước chữ của legend
                title=dict(text="Legend Title", font=dict(size=18))  # Kích thước chữ tiêu đề legend
            ),

            scene=dict(
                xaxis=dict(
                    dtick=0.25,  # Khoảng cách giữa các mốc trên trục X là 0.25
                    title=dict(
                        text="X Axis",  # Nội dung tiêu đề của trục X
                        font=dict(size=16)  # Kích thước chữ
                    )
                ),
                yaxis=dict(
                    dtick=0.25,  # Khoảng cách giữa các mốc trên trục Y là 0.25
                    title=dict(
                        text="Y Axis",  # Nội dung tiêu đề của trục Y
                        font=dict(size=16)  # Kích thước chữ
                    )
                ),
                zaxis=dict(
                    dtick=0.25,  # Khoảng cách giữa các mốc trên trục Z là 0.25
                    title=dict(
                        text="Z Axis",  # Nội dung tiêu đề của trục Z
                        font=dict(size=16)  # Kích thước chữ
                    )
                ),
                aspectmode='data'  # Tỷ lệ dựa trên phạm vi dữ liệu
            )
        )
        # Hiển thị biểu đồ
        fig.show(renderer="browser")

        if save_plot:
            # Lưu biểu đồ thành HTML
            plot(fig, filename=f"{title}_trajectory_plot.html")
            print(f"Trajectory plot was saved at {title}_trajectory_plot.html")

    def update_plot(self, one_trajectory_prediction, one_trajectory_label, color_idx, ax, rotate_data_whose_y_up=False):
        # Check if the color index is within the range
        if color_idx+1 >= len(self.colors):
            return

        # Determine axis order based on rotate_data_whose_y_up flag
        if rotate_data_whose_y_up:
            x_idx, y_idx, z_idx = 0, 2, 1  # Swap y and z
            x_mul, y_mul, z_mul = 1, -1, 1  # Invert y
        else:
            x_idx, y_idx, z_idx = 0, 1, 2  # Default order
            x_mul, y_mul, z_mul = 1, 1, 1

        # Plot actual and predicted trajectories
        ax.plot(one_trajectory_label[:, x_idx]*x_mul, one_trajectory_label[:, y_idx]*y_mul, one_trajectory_label[:, z_idx]*z_mul,
                    'o', color=self.colors[color_idx], alpha=0.5, label=f'Test {color_idx + 1} Actual Trajectory', markersize=10)
        ax.plot(one_trajectory_prediction[:, x_idx]*x_mul, one_trajectory_prediction[:, y_idx]*y_mul, one_trajectory_prediction[:, z_idx]*z_mul,
                    '+', color=self.colors[color_idx], label=f'Test {color_idx + 1} Predicted Trajectory', markersize=10)

        # Calculate limits
        all_data = np.concatenate((one_trajectory_label, one_trajectory_prediction), axis=0)
        ax.set_xlim([all_data[:, x_idx].min(), all_data[:, x_idx].max()])
        ax.set_ylim([all_data[:, y_idx].min(), all_data[:, y_idx].max()])
        ax.set_zlim([all_data[:, z_idx].min(), all_data[:, z_idx].max()])

        # Set labels based on axis order
        ax.set_xlabel('X')
        ax.set_ylabel('Z' if rotate_data_whose_y_up else 'Y')
        ax.set_zlabel('Y' if rotate_data_whose_y_up else 'Z')

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

    def plot_predictions_plt(self, predictions, test_labels, lim_plot_num=None, rotate_data_whose_y_up=False, title=''):
        if lim_plot_num:
            predictions = predictions[:lim_plot_num]
            test_labels = test_labels[:lim_plot_num]
        lim_plot_num = min(len(self.colors), len(predictions))
        figure = plt.figure()
        # set size of figure
        figure.set_size_inches(12, 12)
        ax = figure.add_subplot(111, projection='3d')
        for i in range (lim_plot_num):
            self.update_plot(predictions[i], test_labels[i], i, ax, rotate_data_whose_y_up=rotate_data_whose_y_up)

        plt.legend()
        plt.title('3D Predictions ' + title, fontsize=18)
        plt.show()

    def plot_samples_rviz(self, segments_plot, title, topic_name, frame_id='world'):
        import rospy
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
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

    def plot_line_chart(self, y_values, x_values=None, legends=None, title="Line Chart", x_label="X-axis", y_label="Y-axis", 
                    save_plot=None, x_tick_distance=None, y_tick_distance=None,
                    font_size_title=35, font_size_label=35, font_size_tick=25,
                    keep_source_order=False, y_stds=None, std_display_mode="fill"):
        """
        Vẽ biểu đồ line chart với nhiều đường đồ thị, tùy chọn khoảng cách tick và lưu dưới dạng file HTML.
        
        Args:
            x_values (list): Danh sách giá trị trên trục X.
            y_values (list of list): Danh sách các chuỗi giá trị trên trục Y.
            legends (list): Tên các đường đồ thị. Nếu None, sử dụng mặc định "Series 1", "Series 2", ...
            title (str): Tiêu đề của biểu đồ.
            x_label (str): Nhãn trục X.
            y_label (str): Nhãn trục Y.
            save_plot (str): Đường dẫn lưu file HTML. Nếu None, không lưu.
            x_tick_distance (float): Khoảng cách giữa các tick trên trục X.
            y_tick_distance (float): Khoảng cách giữa các tick trên trục Y.
            font_size_title (int): Kích thước font của tiêu đề.
            font_size_label (int): Kích thước font của nhãn trục.
            font_size_tick (int): Kích thước font của giá trị tick trên trục.
            keep_source_order (bool): Giữ nguyên thứ tự của x_values. Mặc định là False.
            y_stds (list of list): Độ lệch chuẩn tương ứng với y_values. Nếu None, không vẽ sai số.
            std_display_mode (str): Cách hiển thị độ lệch chuẩn: "bar" (thanh) hoặc "fill" (đổ bóng). Mặc định là "fill".
        """
        if x_values is None:
            # create x_values if not provided with np.arange
            x_values = np.arange(len(y_values[0]))
        elif not all(len(x_values) == len(y) for y in y_values):
            raise ValueError("Độ dài của x_values phải bằng với từng chuỗi trong y_values.")
        
        if legends is None:
            legends = [f"Series {i+1}" for i in range(len(y_values))]

        if len(legends) != len(y_values):
            raise ValueError("The number of legends must equal the number of strings in y_values.")
        
        if y_stds is not None and len(y_stds) != len(y_values):
            raise ValueError("The number of y_stds must equal the number of y_values.")
        
        # Nếu giữ thứ tự, ép kiểu x_values thành string
        if keep_source_order:
            x_values = [str(x) for x in x_values]
        
        fig = go.Figure()
        
        for i, (y, legend) in enumerate(zip(y_values, legends)):
            # Thêm đường dữ liệu chính
            fig.add_trace(go.Scatter(
                x=x_values,
                y=y,
                mode='lines+markers',  
                name=legend  
            ))
            
            # Thêm error bars nếu có y_stds
            if y_stds:
                y_std = y_stds[i]
                if std_display_mode == "fill":  # Hiển thị đổ bóng
                    fig.add_trace(go.Scatter(
                        x=x_values,
                        y=[y_val + std for y_val, std in zip(y, y_std)],  # Y trên (upper bound)
                        mode='lines',
                        line=dict(width=0),  # Ẩn đường
                        name=f"{legend} + Std",
                        showlegend=False,
                        hoverinfo="skip",
                        fill=None
                    ))
                    fig.add_trace(go.Scatter(
                        x=x_values,
                        y=[y_val - std for y_val, std in zip(y, y_std)],  # Y dưới (lower bound)
                        mode='lines',
                        line=dict(width=0),  # Ẩn đường
                        name=f"{legend} - Std",
                        showlegend=False,
                        hoverinfo="skip",
                        fill='tonexty',  # Đổ bóng
                        # fillcolor="rgba(255, 0, 0, 0.3)"
                    ))
                elif std_display_mode == "bar":  # Hiển thị bar
                    fig.add_trace(go.Bar(
                        x=x_values,
                        y=[2 * std for std in y_std],  # Chiều cao bar là 2 * độ lệch chuẩn
                        base=[y_val - std for y_val, std in zip(y, y_std)],  # Tọa độ y bắt đầu của bar
                        width=0.1,  # Độ rộng bar (điều chỉnh tùy ý)
                        name=f"{legend} Std",
                        showlegend=False,  # Không hiển thị trong legend
                        # màu blue nhạt với độ trong suốt 0.5
                        marker_color='rgba(0, 0, 255, 0.5)',  # Màu bar (tùy chọn)
                        # marker_color='rgba(0, 0, 0, 0.2)',  # Màu bar (tùy chọn)
                        opacity=0.5  # Độ trong suốt (tùy chỉnh)
                    ))
        
        # Cấu hình tiêu đề, nhãn trục, và kích thước font
        xaxis_config = {
            "title": dict(
                text=x_label,
                font=dict(size=font_size_label)
            ),
            "tickfont": dict(size=font_size_tick),
            "showgrid": True,
        }
        if keep_source_order:
            xaxis_config.update({
                "categoryorder": "array",
                "categoryarray": x_values
            })
        else:
            xaxis_config["dtick"] = x_tick_distance

        fig.update_layout(
            title=dict(
                text=title,
                font=dict(size=font_size_title),  # Kích thước font của tiêu đề
                x=0.5  # Căn giữa tiêu đề
            ),
            legend=dict(
                font=dict(
                    size=35  # Kích thước font của legend
                )
            ),
            xaxis=xaxis_config,
            yaxis=dict(
                title=dict(
                    text=y_label,
                    font=dict(size=font_size_label)  # Kích thước font nhãn trục Y
                ),
                tickfont=dict(size=font_size_tick),  # Kích thước font giá trị tick trục Y
                showgrid=True,
                dtick=y_tick_distance  # Khoảng cách tick trên trục Y
            ),
            template="plotly_white"
        )
        
        # Hiển thị biểu đồ
        fig.show()
        
        # Lưu biểu đồ dưới dạng HTML nếu save_plot được cung cấp
        if save_plot:
            # replace all space in title with underscore
            file_name = title.replace(' ', '_')
            file_name = f'{file_name}.html'
            plot(fig, filename=file_name, auto_open=True)
            print(f'The plot is saved as {file_name}')

    def plot_variable_length_line_chart(self, x_y_pairs, legends=None, title="Line Chart", x_label="X-axis", y_label="Y-axis", 
                                        save_plot=None, font_size_title=40, font_size_label=30, font_size_tick=25):
        """
        Vẽ biểu đồ line chart với các cặp (x, y) có độ dài khác nhau.

        Args:
            x_y_pairs (list of tuple): Danh sách các cặp (x, y), trong đó x và y là các danh sách giá trị.
            legends (list): Tên các đường đồ thị. Nếu None, sử dụng mặc định "Series 1", "Series 2", ...
            title (str): Tiêu đề của biểu đồ.
            x_label (str): Nhãn trục X.
            y_label (str): Nhãn trục Y.
            save_plot (str): Đường dẫn lưu file HTML. Nếu None, không lưu.
            font_size_title (int): Kích thước font của tiêu đề.
            font_size_label (int): Kích thước font của nhãn trục.
            font_size_tick (int): Kích thước font của giá trị tick trên trục.
        """
        # Kiểm tra tính hợp lệ của đầu vào
        if not all(isinstance(pair, tuple) and len(pair) == 2 for pair in x_y_pairs):
            raise ValueError("x_y_pairs phải là danh sách các tuple (x, y).")

        if legends is None:
            legends = [f"Series {i+1}" for i in range(len(x_y_pairs))]

        if len(legends) != len(x_y_pairs):
            raise ValueError("The number of legends must equal the number of x_y_pairs.")

        fig = go.Figure()

        for (x, y), legend in zip(x_y_pairs, legends):
            if len(x) != len(y):
                raise ValueError(f"Cặp (x, y) với legend '{legend}' không có cùng độ dài.")

            # Thêm từng đường vào biểu đồ
            fig.add_trace(go.Scatter(
                x=x,
                y=y,
                mode='lines+markers',  # Đường và marker
                name=legend
            ))

        # Cấu hình biểu đồ
        fig.update_layout(
            title=dict(
                text=title,
                font=dict(size=font_size_title),
                x=0.5  # Căn giữa tiêu đề
            ),
            xaxis=dict(
                title=dict(
                    text=x_label,
                    font=dict(size=font_size_label)
                ),
                tickfont=dict(size=font_size_tick),
                showgrid=True
            ),
            yaxis=dict(
                title=dict(
                    text=y_label,
                    font=dict(size=font_size_label)
                ),
                tickfont=dict(size=font_size_tick),
                showgrid=True
            ),
            template="plotly_white"
        )

        # Hiển thị biểu đồ
        fig.show()

        # Lưu biểu đồ dưới dạng HTML nếu cần
        if save_plot:
            file_name = title.replace(' ', '_') + ".html"
            plot(fig, filename=file_name, auto_open=True)
            print(f'The plot is saved as {file_name}')


    # def plot_bar_chart(self, x_values, y_values, legends=None, title="Bar Chart", x_label="X-axis", y_label="Y-axis", 
    #                 save_plot=None, x_tick_distance=None, y_tick_distance=None,
    #                 font_size_title=20, font_size_label=15, font_size_tick=12,
    #                 keep_source_order=False, y_stds=None, bar_width=0.8):
    #     """
    #     Vẽ biểu đồ bar chart với nhiều cột, tùy chọn vẽ standard deviation (đoạn thẳng có chắn 2 đầu).
        
    #     Args:
    #         x_values (list): Danh sách giá trị trên trục X.
    #         y_values (list of list): Danh sách các chuỗi giá trị trên trục Y.
    #         legends (list): Tên các chuỗi dữ liệu. Nếu None, sử dụng mặc định "Series 1", "Series 2", ...
    #         title (str): Tiêu đề của biểu đồ.
    #         x_label (str): Nhãn trục X.
    #         y_label (str): Nhãn trục Y.
    #         save_plot (str): Đường dẫn lưu file HTML. Nếu None, không lưu.
    #         x_tick_distance (float): Khoảng cách giữa các tick trên trục X.
    #         y_tick_distance (float): Khoảng cách giữa các tick trên trục Y.
    #         font_size_title (int): Kích thước font của tiêu đề.
    #         font_size_label (int): Kích thước font của nhãn trục.
    #         font_size_tick (int): Kích thước font của giá trị tick trên trục.
    #         keep_source_order (bool): Giữ nguyên thứ tự của x_values. Mặc định là False.
    #         y_stds (list of list): Độ lệch chuẩn tương ứng với y_values. Nếu None, không vẽ sai số.
    #         bar_width (float): Độ rộng của bar (từ 0.1 đến 1). Mặc định là 0.8.
    #     """
    #     if not all(len(x_values) == len(y) for y in y_values):
    #         raise ValueError("Độ dài của x_values phải bằng với từng chuỗi trong y_values.")
        
    #     if legends is None:
    #         legends = [f"Series {i+1}" for i in range(len(y_values))]

    #     if len(legends) != len(y_values):
    #         raise ValueError("The number of legends must equal the number of strings in y_values.")
        
    #     if y_stds is not None and len(y_stds) != len(y_values):
    #         raise ValueError("The number of y_stds must equal the number of y_values.")
        
    #     # Nếu giữ thứ tự, ép kiểu x_values thành string
    #     if keep_source_order:
    #         x_values = [str(x) for x in x_values]
        
    #     fig = go.Figure()
        
    #     for i, (y, legend) in enumerate(zip(y_values, legends)):
    #         # Thêm bar chính
    #         fig.add_trace(go.Bar(
    #             x=x_values,
    #             y=y,
    #             name=legend,
    #             width=bar_width  # Điều chỉnh độ rộng của bar
    #         ))
            
    #         # # Thêm giá trị trên đầu mỗi cột
    #         # for x, y_val in zip(x_values, y):
    #         #     fig.add_annotation(
    #         #         x=x,
    #         #         y=y_val,
    #         #         text=f"{y_val:.2f}",  # Format giá trị
    #         #         showarrow=False,  # Không hiển thị mũi tên
    #         #         font=dict(size=font_size_tick),  # Kích thước font
    #         #         xanchor='center',  # Căn giữa text theo trục X
    #         #         yanchor='bottom'  # Hiển thị text phía trên bar
    #         #     )
            
    #         # Thêm đoạn thẳng dọc và chắn trên/dưới cho standard deviation nếu có
    #         if y_stds:
    #             y_std = y_stds[i]
    #             for x, y_val, std in zip(x_values, y, y_std):
    #                 # Đoạn thẳng dọc cho standard deviation
    #                 fig.add_trace(go.Scatter(
    #                     x=[x, x],  # X giữ nguyên (không thay đổi giá trị)
    #                     y=[y_val - std, y_val + std],  # Giá trị y cho đoạn dọc
    #                     mode="lines",
    #                     line=dict(color="black", width=2),  # Đường màu đen, dày 2px
    #                     showlegend=False
    #                 ))
                    
    #                 # Chắn trên
    #                 fig.add_trace(go.Scatter(
    #                     x=[x],  # X giữ nguyên
    #                     y=[y_val + std],  # Giá trị y là đầu trên
    #                     mode="markers",
    #                     marker=dict(symbol="line-ew", size=10, color="black"),  # Dạng chắn ngang
    #                     showlegend=False
    #                 ))
                    
    #                 # Chắn dưới
    #                 fig.add_trace(go.Scatter(
    #                     x=[x],  # X giữ nguyên
    #                     y=[y_val - std],  # Giá trị y là đầu dưới
    #                     mode="markers",
    #                     marker=dict(symbol="line-ew", size=10, color="black"),  # Dạng chắn ngang
    #                     showlegend=False
    #                 ))
        
    #     # Cấu hình tiêu đề, nhãn trục, và kích thước font
    #     xaxis_config = {
    #         "title": dict(
    #             text=x_label,
    #             font=dict(size=font_size_label)
    #         ),
    #         "tickfont": dict(size=font_size_tick),
    #         "showgrid": True,
    #     }
    #     if keep_source_order:
    #         xaxis_config.update({
    #             "categoryorder": "array",
    #             "categoryarray": x_values
    #         })
    #     else:
    #         xaxis_config["dtick"] = x_tick_distance

    #     fig.update_layout(
    #         title=dict(
    #             text=title,
    #             font=dict(size=font_size_title),  # Kích thước font của tiêu đề
    #             x=0.5  # Căn giữa tiêu đề
    #         ),
    #         xaxis=xaxis_config,
    #         yaxis=dict(
    #             title=dict(
    #                 text=y_label,
    #                 font=dict(size=font_size_label)  # Kích thước font nhãn trục Y
    #             ),
    #             tickfont=dict(size=font_size_tick),  # Kích thước font giá trị tick trên trục Y
    #             showgrid=True,
    #             dtick=y_tick_distance  # Khoảng cách tick trên trục Y
    #         ),
    #         template="plotly_white",
    #         barmode='group'  # Các bar được nhóm theo legend
    #     )
        
    #     # Hiển thị biểu đồ
    #     fig.show()
        
    #     # Lưu biểu đồ dưới dạng HTML nếu save_plot được cung cấp
    #     if save_plot:
    #         # replace all space in title with underscore
    #         file_name = title.replace(' ', '_')
    #         file_name = f'{file_name}.html'
    #         plot(fig, filename=file_name, auto_open=True)
    #         print(f'The plot is saved as {file_name}')

    # def plot_bar_chart(self, x_values, y_values, legends=None, title="Bar Chart", x_label="X-axis", y_label="Y-axis", 
    #                save_plot=None, font_size_title=20, font_size_label=15, font_size_tick=12, font_size_bar_val=12, y_stds=None, bar_width=0.8):
    #     """
    #     Vẽ biểu đồ bar chart với nhiều cột, tùy chọn vẽ standard deviation (đoạn thẳng có chắn 2 đầu).
        
    #     Args:
    #         x_values (list): Danh sách giá trị trên trục X.
    #         y_values (list of list): Danh sách các chuỗi giá trị trên trục Y.
    #         legends (list): Tên các chuỗi dữ liệu. Nếu None, sử dụng mặc định "Series 1", "Series 2", ...
    #         title (str): Tiêu đề của biểu đồ.
    #         x_label (str): Nhãn trục X.
    #         y_label (str): Nhãn trục Y.
    #         save_plot (str): Đường dẫn lưu file PNG. Nếu None, không lưu.
    #         font_size_title (int): Kích thước font của tiêu đề.
    #         font_size_label (int): Kích thước font của nhãn trục.
    #         font_size_tick (int): Kích thước font của giá trị tick trên trục.
    #         y_stds (list of list): Độ lệch chuẩn tương ứng với y_values. Nếu None, không vẽ sai số.
    #         bar_width (float): Độ rộng của bar (từ 0.1 đến 1). Mặc định là 0.8.
    #     """
    #     if legends is None:
    #         # legends = [f"Series {i+1}" for i in range(len(y_values))]
    #         legends = ['']
        
    #     # Kiểm tra độ dài dữ liệu
    #     if not all(len(x_values) == len(y) for y in y_values):
    #         raise ValueError("Độ dài của x_values phải bằng với từng chuỗi trong y_values.")
        
    #     if y_stds is not None and len(y_stds) != len(y_values):
    #         raise ValueError("The number of y_stds must equal the number of y_values.")
        
    #     x_indices = np.arange(len(x_values))  # Vị trí các cột trên trục X

    #     # Khởi tạo plot
    #     fig, ax = plt.subplots(figsize=(20, 12))
        
    #     # Vẽ các bar và thêm giá trị phía trên
    #     for i, y in enumerate(y_values):
    #         offset_x = (i - (len(y_values) - 1) / 2) * bar_width  # Điều chỉnh vị trí các nhóm cột
    #         ax.bar(x_indices + offset_x, y, bar_width, label=legends[i],
    #             yerr=y_stds[i] if y_stds else None, capsize=5)  # Vẽ cột và lỗi chuẩn
            
    #         # Thêm giá trị phía trên cột, hiển thị giá trị màu đỏ
    #         for x, y_val in zip(x_indices + offset_x, y):
    #             ax.text(x, y_val, f"{y_val:.5f}", ha='center', va='bottom', fontsize=font_size_bar_val, color='red')
    #             # input(y_val)


    #     # Cấu hình trục X và Y
    #     ax.set_xticks(x_indices)
    #     ax.set_xticklabels(x_values, fontsize=font_size_tick)
    #     ax.set_xlabel(x_label, fontsize=font_size_label)
    #     ax.set_ylabel(y_label, fontsize=font_size_label, labelpad=50)
    #     ax.set_title(title, fontsize=font_size_title)
        
    #     # Hiển thị lưới và chú thích
    #     ax.grid(axis='y', linestyle='--', alpha=0.7)
    #     ax.legend(fontsize=font_size_tick)

    #     # Lưu biểu đồ nếu cần
    #     if save_plot:
    #         plt.savefig(save_plot, dpi=300, bbox_inches='tight')
    #         print(f"Biểu đồ đã được lưu tại: {save_plot}")
        
    #     # Hiển thị biểu đồ
    #     plt.tight_layout()
    #     plt.show()

    def plot_bar_chart(self, x_values, y_values, legends=None, title="Bar Chart", x_label="X-axis", y_label="Y-axis", 
                   save_plot=None, font_size_title=20, font_size_label=15, font_size_tick=12, font_size_bar_val=12, 
                   y_stds=None, bar_width=0.8, y_lim=None):
        """
        Vẽ biểu đồ bar chart với nhiều cột, tùy chọn vẽ standard deviation (đoạn thẳng có chắn 2 đầu).
        
        Args:
            x_values (list): Danh sách giá trị trên trục X.
            y_values (list of list): Danh sách các chuỗi giá trị trên trục Y.
            legends (list): Tên các chuỗi dữ liệu. Nếu None, sử dụng mặc định "Series 1", "Series 2", ...
            title (str): Tiêu đề của biểu đồ.
            x_label (str): Nhãn trục X.
            y_label (str): Nhãn trục Y.
            save_plot (str): Đường dẫn lưu file PNG. Nếu None, không lưu.
            font_size_title (int): Kích thước font của tiêu đề.
            font_size_label (int): Kích thước font của nhãn trục.
            font_size_tick (int): Kích thước font của giá trị tick trên trục.
            y_stds (list of list): Độ lệch chuẩn tương ứng với y_values. Nếu None, không vẽ sai số.
            bar_width (float): Độ rộng của bar (từ 0.1 đến 1). Mặc định là 0.8.
            y_lim (tuple): Giới hạn trục Y dưới dạng (min, max). Nếu None, tự động điều chỉnh.
        """
        if legends is None:
            legends = ['']
        
        # Kiểm tra độ dài dữ liệu
        if not all(len(x_values) == len(y) for y in y_values):
            raise ValueError("Độ dài của x_values phải bằng với từng chuỗi trong y_values.")
        
        if y_stds is not None and len(y_stds) != len(y_values):
            raise ValueError("The number of y_stds must equal the number of y_values.")
        
        x_indices = np.arange(len(x_values))  # Vị trí các cột trên trục X

        # Khởi tạo plot
        fig, ax = plt.subplots(figsize=(20, 12))
        
        # Vẽ các bar và thêm giá trị phía trên
        for i, y in enumerate(y_values):
            offset_x = (i - (len(y_values) - 1) / 2) * bar_width  # Điều chỉnh vị trí các nhóm cột
            ax.bar(x_indices + offset_x, y, bar_width, label=legends[i],
                yerr=y_stds[i] if y_stds else None, capsize=5)  # Vẽ cột và lỗi chuẩn
            
            # Thêm giá trị phía trên cột, hiển thị giá trị màu đỏ
            for x, y_val in zip(x_indices + offset_x, y):
                ax.text(x, y_val, f"{y_val:.5f}", ha='center', va='bottom', fontsize=font_size_bar_val, color='red')

        # Cấu hình trục X và Y
        ax.set_xticks(x_indices)
        ax.set_xticklabels(x_values, fontsize=font_size_tick)
        ax.set_xlabel(x_label, fontsize=font_size_label)
        ax.set_ylabel(y_label, fontsize=font_size_label, labelpad=50)
        ax.set_title(title, fontsize=font_size_title)

        # Thiết lập giới hạn trục Y nếu được chỉ định
        if y_lim is not None:
            ax.set_ylim(y_lim)

        # Hiển thị lưới và chú thích
        ax.grid(axis='y', linestyle='--', alpha=0.7)
        ax.legend(fontsize=font_size_tick)

        # Lưu biểu đồ nếu cần
        if save_plot:
            plt.savefig(save_plot, dpi=300, bbox_inches='tight')
            print(f"Biểu đồ đã được lưu tại: {save_plot}")

        # Hiển thị biểu đồ
        plt.tight_layout()
        plt.show()

    
    def draw_histogram(self, data, bin_width, x_label, y_label, title, start_x=None, end_x=None):
        # Thiết lập kích thước cửa sổ ngay khi bắt đầu
        fig, ax = plt.subplots(figsize=(20, 12))
        
        # Tính min và max của dữ liệu
        min_value = min(data)
        max_value = max(data)

        # Tạo các bin theo bin_width
        bins = [min_value + i * bin_width for i in range(int((max_value - min_value) / bin_width) + 1)]

        # Tính bin start
        if start_x is not None and start_x < min_value:
            bin_start = np.floor(start_x / bin_width) * bin_width
        else:
            bin_start = np.floor(min_value / bin_width) * bin_width

        # Tính bin end
        if end_x is not None and end_x > max_value:
            bin_end = np.ceil(end_x / bin_width) * bin_width + bin_width
        else:
            bin_end = np.ceil(max_value / bin_width) * bin_width + bin_width

        bins = np.arange(bin_start, bin_end, bin_width)

        # Vẽ histogram
        ax.hist(data, bins=bins, edgecolor='black', alpha=0.7)

        # Ghi giá trị y lên từng bar
        for i in range(len(bins) - 1):
            count = sum(1 for x in data if bins[i] <= x < bins[i + 1])  # Tính tần suất của mỗi bin
            ax.text((bins[i] + bins[i + 1]) / 2, count, str(count), ha='center', va='bottom', fontsize=10, color='red')

        # Tùy chỉnh trục x
        if start_x is not None and end_x is not None:
            ax.set_xlim(start_x, end_x)

        ax.set_xticks(bins)

        # Thiết lập các thuộc tính của biểu đồ
        ax.set_xlabel(x_label, fontsize=14, labelpad=20)
        ax.set_ylabel(y_label, fontsize=14, labelpad=20)
        ax.set_title(title + f' - Total data number: {len(data)}', fontsize=20, pad=30)
        ax.grid(True)

        # Hiển thị biểu đồ
        plt.show()


def main():
    util_plotter = Plotter()

    # Test plot_trajectory_dataset
    #   load data
    #   get path of this script
    this_path = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(this_path, 'data/data_test_27.npy')
    object_name = '27'

    data = np.load(data_path, allow_pickle=True)
    print(len(data))
    trajectories = data[:10]
    util_plotter.plot_trajectory_dataset(trajectories, title=object_name, rotate_data_whose_y_up=False, save_plot=False)

if __name__ == '__main__':
    main()
