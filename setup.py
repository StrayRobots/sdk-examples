from setuptools import setup, find_packages

setup(
    name="straypublic",
    version="0.0.1",
    author="Stray Robots",
    author_email="julius@strayrobots.io",
    description="Stray Robots public scripts.",
    url="https://strayrobots.io",
    classifiers=[
        "Programming Language :: Python :: 3",
    ],
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=["torchvision", "torch", "open3d", "numpy", "opencv-python", "pyrealsense2"]
)
