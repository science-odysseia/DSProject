from setuptools import find_packages, setup

package_name = "pipe_inspect_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dongyeon",
    maintainer_email="loik1235@gmail.com",
    description="배관 점검 로봇 — 활성 카메라 영상·OpenCV 판정 수신, 결함 리포트 적재와 수리 판정 (ROS 노드 쪽)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "view_active_cam = pipe_inspect_demo.view_active_cam_node:main",
            "pipe_report = pipe_inspect_demo.pipe_report_node:main",
            "pipe_coordinator = pipe_inspect_demo.pipe_coordinator_node:main",
            "repair_target_test = pipe_inspect_demo.repair_target_test_node:main",
        ],
    },
)
