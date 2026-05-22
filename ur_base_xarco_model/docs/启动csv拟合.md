一级标题 <Alt+Ctrl+1>二级标题 <Alt+Ctrl+2>三级标题 <Alt+Ctrl+3>四级标题 <Alt+Ctrl+4>五级标题 <Alt+Ctrl+5>六级标题 <Alt+Ctrl+6>

pkill -f replay_csv.launch.py || true
pkill -f csv_joint_replay.py || true

cd /root/ur10_ws
colcon build --packages-select assembly_description --symlink-install
source install/setup.bash
ros2 launch assembly_description replay_csv.launch.py loop:=true publish_hz:=125.0 speed_scale:=1.0

[ ]

所见即所得 <Alt+Ctrl+7>即时渲染 <Alt+Ctrl+8>分屏预览 <Alt+Ctrl+9>

大纲

DesktopTabletMobile/Wechat
