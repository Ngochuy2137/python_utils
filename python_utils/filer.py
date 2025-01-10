import os

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
