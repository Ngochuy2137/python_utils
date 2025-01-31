import os
import re

def find_directories_with_keyword(directory, keyword):
    """
    Tìm kiếm các thư mục trong một đường dẫn chứa từ khóa trong tên 
    (không phân biệt chữ hoa/chữ thường, tìm cả chuỗi con).

    Args:
        directory (str): Đường dẫn thư mục gốc để tìm kiếm.
        keyword (str): Từ khóa cần tìm trong tên thư mục.

    Returns:
        list: Danh sách các đường dẫn thư mục có chứa từ khóa.
    """
    matching_paths = []
    keyword_lower = keyword.lower()  # Chuyển từ khóa về chữ thường
    for root, dirs, files in os.walk(directory):
        for dir_name in dirs:
            # So sánh không phân biệt chữ hoa/chữ thường, tìm chuỗi con
            if keyword_lower in dir_name.lower():
                matching_paths.append(os.path.join(root, dir_name))
    return matching_paths

# def list_subfolders(root_path, only_folder_name=False):
#     """
#     Lấy danh sách các thư mục con của root_path.
#     Args:
#         root_path (str): Đường dẫn tới thư mục gốc.
#     Returns:
#         list: Danh sách các thư mục con.
#     """
#     subfolders = [f.path for f in os.scandir(root_path) if f.is_dir()]
#     if only_folder_name:
#         subfolders = [os.path.basename(f) for f in subfolders]
#     return subfolders

def list_subfolders(root_path, only_folder_name=False, pattern=None):
    """
    Lấy danh sách các thư mục con của root_path với tùy chọn lọc theo điều kiện.

    Args:
        root_path (str): Đường dẫn tới thư mục gốc.
        only_folder_name (bool): Nếu True, chỉ trả về tên thư mục, ngược lại trả về đường dẫn đầy đủ.
        pattern (str, optional): Biểu thức chính quy để lọc thư mục theo mẫu.

    Returns:
        list: Danh sách các thư mục con (đã lọc nếu có điều kiện).
    """
    subfolders = [f.path for f in os.scandir(root_path) if f.is_dir()]
    # Lọc theo mẫu regex nếu được cung cấp
    if pattern:
        subfolders = [f for f in subfolders if re.match(pattern, os.path.basename(f))]
    if only_folder_name:
        subfolders = [os.path.basename(f) for f in subfolders]

    return subfolders

def list_files(root_path, only_file_name=False, pattern=None):
    """
    Lấy danh sách các tệp tin trong root_path với tùy chọn lọc theo điều kiện.

    Args:
        root_path (str): Đường dẫn tới thư mục gốc.
        only_file_name (bool): Nếu True, chỉ trả về tên tệp, ngược lại trả về đường dẫn đầy đủ.
        pattern (str, optional): Biểu thức chính quy để lọc tên tệp tin theo mẫu.

    Returns:
        list: Danh sách các tệp tin (đã lọc nếu có điều kiện).
    """
    # Lấy danh sách tất cả các tệp tin trong thư mục
    files = [f.path for f in os.scandir(root_path) if f.is_file()]

    # Nếu cần lọc theo pattern (biểu thức chính quy)
    if pattern:
        files = [f for f in files if re.match(pattern, os.path.basename(f))]

    # Nếu chỉ lấy tên tệp thay vì đường dẫn đầy đủ
    if only_file_name:
        files = [os.path.basename(f) for f in files]

    return sorted(files)