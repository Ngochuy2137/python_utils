# setup.py
from setuptools import setup, find_packages

setup(
    name="python_utils",
    version="0.1",
    packages=find_packages(),
    install_requires=[],  # Các thư viện cần thiết nếu có
    author="Huynn",
    author_email="huy.nguyenngoc2137@gmail.com",
    description="Python utilities",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/Ngochuy2137/python_utils",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)