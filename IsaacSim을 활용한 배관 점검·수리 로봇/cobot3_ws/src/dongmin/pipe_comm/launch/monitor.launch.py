"""카메라·주행 감시를 한 번에 띄운다.

  ros2 launch pipe_comm monitor.launch.py
  ros2 launch pipe_comm monitor.launch.py ns:=elbow_v
  ros2 launch pipe_comm monitor.launch.py ns:=all          # 코스 3종 전부

🔑 `ns:=all` 이면 주행 감시는 **한 노드가 3대를 같이** 보고, 카메라 감시는
   로봇마다 하나씩 뜬다(영상 통계는 섞으면 못 읽는다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pipe_comm import contract


def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration("ns").perform(context)
    period = float(LaunchConfiguration("period_sec").perform(context))
    names = list(contract.COURSE_NS) if ns == "all" else [ns]

    nodes = [Node(package="pipe_comm", executable="drive_monitor",
                  name="drive_monitor", output="screen",
                  parameters=[{"ns": names, "period_sec": period}])]
    nodes += [Node(package="pipe_comm", executable="camera_monitor",
                   name=f"camera_monitor_{n}", output="screen",
                   parameters=[{"ns": n, "period_sec": period}])
              for n in names]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value=contract.DEFAULT_NS,
                              description="로봇 네임스페이스. 'all' 이면 코스 3종"),
        DeclareLaunchArgument("period_sec", default_value="1.0",
                              description="통계 출력 주기(초)"),
        OpaqueFunction(function=_setup),
    ])
