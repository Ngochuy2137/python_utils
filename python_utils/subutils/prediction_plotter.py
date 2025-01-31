from plotly.graph_objects import Figure, Scatter3d
import numpy as np
import random
from plotly.offline import plot
import plotly.io as pio

class PredictionPlotter:
    def __init__(self):
        pass
    def _generate_colors(self, n):
        """Tạo danh sách màu sắc ngẫu nhiên."""
        self.colors = [
            f'rgb({random.randint(0,255)}, {random.randint(0,255)}, {random.randint(0,255)})'
            for _ in range(n)
        ]

    def _plot_scatter(self, x, y, z, marker_size, marker_symbol, color, name, legendgroup, line_width=2, opacity=0.8):
        """Hàm con để vẽ một tập hợp điểm trong Scatter3d với tùy chỉnh viền và độ mờ."""
        return Scatter3d(
            x=x,
            y=y,
            z=z,
            mode='markers',
            marker=dict(
                size=marker_size,
                color=color,
                symbol=marker_symbol,
                opacity=opacity,  # Điều chỉnh độ mờ
                line=dict(
                    width=line_width,  # Điều chỉnh độ dày viền
                    color=color  # Màu của viền
                )
            ),
            name=name,
            legendgroup=legendgroup
        )

    def _plot_dashed_line(self, start, end, color, legendgroup):
        """Hàm con để vẽ một đường đứt đoạn nối hai điểm."""
        return Scatter3d(
            x=[start[0], end[0]],
            y=[start[1], end[1]],
            z=[start[2], end[2]],
            mode='lines',
            line=dict(
                color=color,
                width=2,
                dash='dash'
            ),
            showlegend=False,
            legendgroup=legendgroup  # Gán nhóm legend
        )

    def _plot_text(self, x, y, z, text, color, size, legendgroup):
        """Hàm con để thêm văn bản vào đồ thị."""
        return Scatter3d(
            x=[x],
            y=[y],
            z=[z],
            mode='text',
            text=[text],
            textfont=dict(size=size, color=color),
            showlegend=False,
            legendgroup=legendgroup  # Gán nhóm legend
        )

    def create_figure(self, inputs, labels, predictions, rotate_data_whose_y_up=False, title='', font_size_note=10, show_all_as_default=False):
        fig = Figure()

        # Tạo danh sách màu sắc
        n_trajectories = len(predictions)
        self._generate_colors(n_trajectories)

        # Thiết lập trạng thái hiển thị mặc định
        initial_visibility = True if show_all_as_default else "legendonly"

        for i, (input_traj, label_traj, pred_traj) in enumerate(zip(inputs, labels, predictions)):
            # legendgroup = f'Trajectory {i+1}'  # Nhóm legend cho từng quỹ đạo

            def process_trajectory(traj):
                if rotate_data_whose_y_up:
                    return traj[:, 0], -traj[:, 2], traj[:, 1]
                return traj[:, 0], traj[:, 1], traj[:, 2]

            input_x, input_y, input_z = process_trajectory(input_traj)
            label_x, label_y, label_z = process_trajectory(label_traj)
            pred_x, pred_y, pred_z = process_trajectory(pred_traj)

            # Tính khoảng cách giữa các điểm label và predict tương ứng
            distances = [
                np.linalg.norm(np.array([px, py, pz]) - np.array([lx, ly, lz]))
                for px, py, pz, lx, ly, lz in zip(pred_x, pred_y, pred_z, label_x, label_y, label_z)
            ]

            # Tạo thông tin hover bổ sung cho predictions
            in_hover = [
                f"IN<br>  ID: {idx}<br>  X: {px:.2f}<br>  Y: {py:.2f}<br>  Z: {pz:.2f}"
                for idx, (px, py, pz) in enumerate(zip(input_x, input_y, input_z))
            ]

            pred_hover = [
                f"PRED<br>  ID: {idx}<br>  X: {px:.2f}<br>  Y: {py:.2f}<br>  Z: {pz:.2f}<br>  err: {dist:.4f}"
                for idx, (px, py, pz, dist) in enumerate(zip(pred_x, pred_y, pred_z, distances))
            ]

            # Tạo thông tin hover bổ sung cho labels
            label_hover = [
                f"LAB<br>  ID: {idx}<br>  X: {lx:.2f}<br>  Y: {ly:.2f}<br>  Z: {lz:.2f}<br>  err: {dist:.4f}"
                for idx, (lx, ly, lz, dist) in enumerate(zip(label_x, label_y, label_z, distances))
            ]

            # Vẽ inputs
            fig.add_trace(self._plot_scatter(input_x, input_y, input_z, 12, 'circle-open', self.colors[i], name=f'Input {i} ({len(input_x)})', legendgroup=f'Input {i}', line_width=2)
                        .update(visible=initial_visibility, hovertext=in_hover, hoverinfo='text'))

            # Vẽ labels với hovertext
            fig.add_trace(self._plot_scatter(label_x, label_y, label_z, 6, 'circle', self.colors[i], name=f'Label {i} ({len(label_x)})', legendgroup=f'Label {i}', line_width=1, opacity=0.5)
                        .update(visible=initial_visibility, hovertext=label_hover, hoverinfo='text'))

            # Vẽ predictions với hovertext
            fig.add_trace(self._plot_scatter(pred_x, pred_y, pred_z, 4, 'cross', self.colors[i], name=f'Prediction {i} ({len(pred_x)})', legendgroup=f'Prediction {i}', line_width=1)
                        .update(visible=initial_visibility, hovertext=pred_hover, hoverinfo='text'))

            # Tính điểm cuối của predictions và labels
            pred_end = np.array([pred_x[-1], pred_y[-1], pred_z[-1]])
            label_end = np.array([label_x[-1], label_y[-1], label_z[-1]])

            # Vẽ đường đứt đoạn nối prediction và label cuối
            fig.add_trace(self._plot_dashed_line(pred_end, label_end, self.colors[i], legendgroup=f'end {i}')
                        .update(visible=initial_visibility, showlegend=True, name=f'end (P) {i}'))

            # Tính và hiển thị khoảng cách Euclidean giữa prediction và label cuối
            distance = np.linalg.norm(pred_end - label_end)

            # Thêm text "end" cho điểm cuối của predictions
            fig.add_trace(self._plot_text(pred_end[0], pred_end[1], pred_end[2], f"end (P) {i} - err: {distance:.3f}", self.colors[i], 10, legendgroup=f'end {i}')
                        .update(visible=initial_visibility, showlegend=True, name=f'<br>'))

        # Cập nhật bố cục
        fig.update_layout(
            title=dict(
                text=f"{title} - Inputs vs Labels vs Predictions",
                font=dict(size=32),
                x=0.5
            ),
            legend=dict(
                font=dict(size=16),
                title=dict(text="Legend Title", font=dict(size=18))
            ),
            scene=dict(
                xaxis=dict(
                    dtick=0.25,
                    title=dict(
                        text="X Axis",
                        font=dict(size=16)
                    )
                ),
                yaxis=dict(
                    dtick=0.25,
                    title=dict(
                        text="Y Axis",
                        font=dict(size=16)
                    )
                ),
                zaxis=dict(
                    dtick=0.25,
                    title=dict(
                        text="Z Axis",
                        font=dict(size=16)
                    )
                ),
                aspectmode='data',
            ),
        )
        return fig

    # def create_figure(self, inputs=None, labels=None, predictions=None, rotate_data_whose_y_up=False, title='', font_size_note=10, show_all_as_default=False):
    #     fig = Figure()

    #     # Kiểm tra số lượng quỹ đạo từ tham số không phải None
    #     n_trajectories = 0
    #     if predictions is not None:
    #         n_trajectories = len(predictions)
    #     elif labels is not None:
    #         n_trajectories = len(labels)
    #     elif inputs is not None:
    #         n_trajectories = len(inputs)

    #     # Tạo danh sách màu sắc
    #     self._generate_colors(n_trajectories)

    #     # Thiết lập trạng thái hiển thị mặc định
    #     initial_visibility = True if show_all_as_default else "legendonly"

    #     for i in range(n_trajectories):
    #         legendgroup = f'Trajectory {i+1}'  # Nhóm legend cho từng quỹ đạo

    #         def process_trajectory(traj):
    #             if rotate_data_whose_y_up:
    #                 return traj[:, 0], -traj[:, 2], traj[:, 1]
    #             return traj[:, 0], traj[:, 1], traj[:, 2]

    #         # Xử lý và vẽ inputs nếu tồn tại
    #         if inputs is not None:
    #             input_traj = inputs[i]
    #             input_x, input_y, input_z = process_trajectory(input_traj)
    #             fig.add_trace(self._plot_scatter(input_x, input_y, input_z, 12, 'circle-open', self.colors[i], f'Input {i}', legendgroup, line_width=2).update(visible=initial_visibility))

    #         # Xử lý và vẽ labels nếu tồn tại
    #         if labels is not None:
    #             label_traj = labels[i]
    #             label_x, label_y, label_z = process_trajectory(label_traj)
    #             fig.add_trace(self._plot_scatter(label_x, label_y, label_z, 6, 'circle', self.colors[i], f'Label {i}', legendgroup, line_width=1, opacity=0.5).update(visible=initial_visibility))

    #         # Xử lý và vẽ predictions nếu tồn tại
    #         if predictions is not None:
    #             pred_traj = predictions[i]
    #             pred_x, pred_y, pred_z = process_trajectory(pred_traj)
    #             fig.add_trace(self._plot_scatter(pred_x, pred_y, pred_z, 4, 'cross', self.colors[i], f'Prediction {i}', legendgroup, line_width=1).update(visible=initial_visibility))

    #             # Nếu cả labels và predictions tồn tại, tính và vẽ khoảng cách
    #             if labels is not None:
    #                 pred_end = np.array([pred_x[-1], pred_y[-1], pred_z[-1]])
    #                 label_end = np.array([label_x[-1], label_y[-1], label_z[-1]])

    #                 # Vẽ đường đứt đoạn nối prediction và label cuối
    #                 fig.add_trace(self._plot_dashed_line(pred_end, label_end, self.colors[i], legendgroup).update(visible=initial_visibility))

    #                 # Tính và hiển thị khoảng cách Euclidean giữa prediction và label cuối
    #                 distance = np.linalg.norm(pred_end - label_end)
    #                 fig.add_trace(self._plot_text(pred_end[0], pred_end[1], pred_end[2], f"end (P) {i} - err: {distance:.3f} ", self.colors[i], 10, legendgroup).update(visible=initial_visibility))

    #     # Cập nhật bố cục
    #     fig.update_layout(
    #         title=dict(
    #             text=f"{title} - Inputs vs Labels vs Predictions",
    #             font=dict(size=32),
    #             x=0.5
    #         ),
    #         legend=dict(
    #             font=dict(size=16),
    #             title=dict(text="Legend Title", font=dict(size=18))
    #         ),
    #         scene=dict(
    #             xaxis=dict(
    #                 dtick=0.25,
    #                 title=dict(
    #                     text="X Axis",
    #                     font=dict(size=16)
    #                 )
    #             ),
    #             yaxis=dict(
    #                 dtick=0.25,
    #                 title=dict(
    #                     text="Y Axis",
    #                     font=dict(size=16)
    #                 )
    #             ),
    #             zaxis=dict(
    #                 dtick=0.25,
    #                 title=dict(
    #                     text="Z Axis",
    #                     font=dict(size=16)
    #                 )
    #             ),
    #             aspectmode='data',
    #         ),
    #     )
    #     return fig


    def plot_predictions(self, inputs, labels, predictions, rotate_data_whose_y_up=False, title='', save_plot=False, font_size_note=12, show_all_as_default=True):
        fig = self.create_figure(inputs, labels, predictions, rotate_data_whose_y_up, title=title, font_size_note=font_size_note, show_all_as_default=show_all_as_default)
        if save_plot:
            file_name = title + '_inputs_labels_predictions.html'
            plot(fig, filename=file_name, auto_open=True)
            print(f'The plot is saved as {file_name}')
        else:
            pio.show(fig)

def main():
    # Khởi tạo Plotter
    plotter = PredictionPlotter()

    # Dữ liệu giả để demo
    np.random.seed(42)
    n_samples = 10
    inputs = [np.random.rand(20, 3) for _ in range(n_samples)]
    predictions = [np.random.rand(20, 3) for _ in range(n_samples)]
    labels = [np.random.rand(20, 3) for _ in range(n_samples)]

    # Tạo biểu đồ
    plotter.plot_predictions(inputs, labels, predictions, rotate_data_whose_y_up=True, save_plot=False)

if __name__ == '__main__':
    main()
