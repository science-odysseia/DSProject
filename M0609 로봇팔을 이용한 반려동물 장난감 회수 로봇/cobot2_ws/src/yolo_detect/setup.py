from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolo_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'),
            [f for f in glob('resource/*') + glob('resource/.env') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'gui_web', 'templates'),
            glob('yolo_detect/gui_web/templates/*')),
        (os.path.join('share', package_name, 'gui_web', 'static', 'css'),
            glob('yolo_detect/gui_web/static/css/*')),
        (os.path.join('share', package_name, 'gui_web', 'static', 'js'),
            glob('yolo_detect/gui_web/static/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='소파 밑 장난감 자율 탐색·회수 로봇',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_control_node = yolo_detect.robot_control_node:main',
            'object_detect_node = yolo_detect.object_detect_node:main',
            'depth_target_filter_node = yolo_detect.depth_target_filter_node:main',
            'gui_web_node = yolo_detect.gui_web_node:main',
        ],
    },
)
