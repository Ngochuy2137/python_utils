import subprocess
import time

def beep(duration: float = 0.1, freq: float = 440.0) -> None:
    """
    Phát một tiếng beep dùng sox qua ALSA.

    Args:
        duration: độ dài âm thanh (giây), mặc định 0.1s
        freq: tần số (Hz), mặc định 440 Hz (A4)
    """
    try:
        subprocess.run([
            'play', '-nq', '-t', 'alsa',
            'synth', str(duration), 'sine', str(freq)
        ], check=True)
    except FileNotFoundError:
        print("Lỗi: không tìm thấy lệnh 'play'. Hãy cài sox (sudo apt install sox libsox-fmt-all).")
    except subprocess.CalledProcessError as e:
        print(f"Beep thất bại, return code: {e.returncode}")

def warn_beep(iter):
    """
    Phát tiếng beep cảnh báo.
    """
    for _ in range(iter):
        for _ in range(3):
            beep(duration=0.2, freq=440.0)
        # nghỉ 0.5 giây giữa các tiếng beep, không dùng rospy
        time.sleep(0.5)