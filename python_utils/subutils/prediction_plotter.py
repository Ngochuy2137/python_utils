from plotly.graph_objects import Figure, Scatter3d
import numpy as np
import random
from plotly.offline import plot
import plotly.io as pio

class PredictionPlotter:
    def __init__(self):
        self.colors = []

    def _generate_colors(self, n):
        """Tạo danh sách màu sắc ngẫu nhiên."""
        self.colors = [
            f'rgb({random.randint(0,255)}, {random.randint(0,255)}, {random.randint(0,255)})'
            for _ in range(n)
        ]

    def _plot_scatter(self, x, y, z, marker_size, marker_symbol, color, name):
        """Hàm con để vẽ một tập hợp điểm trong Scatter3d."""
        return Scatter3d(
            x=x,
            y=y,
            z=z,
            mode='markers',
            marker=dict(
                size=marker_size,
                color=color,
                symbol=marker_symbol,
                opacity=0.8
            ),
            name=name
        )

    def _plot_dashed_line(self, start, end, color):
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
            showlegend=False
        )

    def create_figure(self, predictions, labels, inputs, rotate_data_whose_y_up=False, notes=''):
        fig = Figure()

        # Tạo danh sách màu sắc
        n_trajectories = len(predictions)
        self._generate_colors(n_trajectories)

        for i, (input_traj, pred_traj, label_traj) in enumerate(zip(inputs, predictions, labels)):
            def process_trajectory(traj):
                if rotate_data_whose_y_up:
                    return traj[:, 0], -traj[:, 2], traj[:, 1]
                return traj[:, 0], traj[:, 1], traj[:, 2]

            input_x, input_y, input_z = process_trajectory(input_traj)
            pred_x, pred_y, pred_z = process_trajectory(pred_traj)
            label_x, label_y, label_z = process_trajectory(label_traj)

            # Vẽ inputs
            fig.add_trace(self._plot_scatter(input_x, input_y, input_z, 5, 'diamond', self.colors[i], f'Input {i+1}'))

            # Vẽ predictions
            fig.add_trace(self._plot_scatter(pred_x, pred_y, pred_z, 6, 'circle', self.colors[i], f'Prediction {i+1}'))

            # Vẽ labels
            fig.add_trace(self._plot_scatter(label_x, label_y, label_z, 4, 'x', self.colors[i], f'Label {i+1}'))

            # Vẽ đường đứt đoạn nối prediction và label cuối
            pred_end = np.array([pred_x[-1], pred_y[-1], pred_z[-1]])
            label_end = np.array([label_x[-1], label_y[-1], label_z[-1]])
            fig.add_trace(self._plot_dashed_line(pred_end, label_end, 'red'))

        # Cập nhật bố cục
        fig.update_layout(
            title=dict(
                text=f"{notes} - Inputs vs Labels vs Predictions",
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
                # Thiết lập tỉ lệ của các trục𝑥x,𝑦y, và𝑧z dựa trên phạm vi của dữ liệu (data) trên từng trục.
                # Điều này đảm bảo rằng các trục có tỉ lệ thực tế chính xác theo dữ liệu.
                aspectmode='data'
            ),
        )
        return fig

    def plot_preditions(self, predictions, labels, inputs, rotate_data_whose_y_up=False, save_html=False):
        fig = self.create_figure(predictions, labels, inputs, rotate_data_whose_y_up)
        if save_html:
            plot(fig, filename='trajectory_plot.html', auto_open=True)
        else:
            pio.show(fig)
        plot(fig, filename='trajectory_plot.html', auto_open=True)

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
    plotter.plot_preditions(predictions, labels, inputs, rotate_data_whose_y_up=True, save_html=False)

if __name__ == '__main__':
    main()
