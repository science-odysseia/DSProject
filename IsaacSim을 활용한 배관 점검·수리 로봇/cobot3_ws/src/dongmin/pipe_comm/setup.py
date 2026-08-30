from glob import glob

from setuptools import find_packages, setup

package_name = "pipe_comm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        # 🔑 web_panel 페이지는 이 패키지 안이 아니라 **워크스페이스의
        #    `web/`** 에 있다 (파이썬이 아니라 웹 소스라 따로 관리한다).
        #    여기서 share/pipe_comm/web 으로 복사해 두면 소스 트리 없이
        #    설치본만 있어도 패널이 뜬다. 경로 해석은 web_panel.find_web().
        ("share/" + package_name + "/web", glob("../../../web/*.*")),
        ("share/" + package_name + "/web/views", glob("../../../web/views/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dongmin",
    maintainer_email="loik1235@gmail.com",
    description="배관 점검 로봇 — Isaac Sim(PC1) ↔ ROS 2(PC2) 통신 규약과 감시·지령 노드",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_monitor = pipe_comm.camera_monitor:main",
            "drive_monitor = pipe_comm.drive_monitor:main",
            "mission_cli = pipe_comm.mission_cli:main",
            "web_view = pipe_comm.web_view:main",
            "web_panel = pipe_comm.web_panel:main",
        ],
    },
)
