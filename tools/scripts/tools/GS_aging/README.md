# GS aging 测试流程

## 准备工作
- 将 RB 和 GS 放在下图相应位置
![](img/img.png) 
- 确认 GS 和本机处于相同网络

- 链接 GS：`ssh augbooth@gspx-00xx.local`
- `cd catkin_ws/src/boothbot/`
- `git checkout develop`
- `git pull`
- `cd catkin_ws/src/augustbot-tools/`
- `git checkout develop`
- `git pull`
- 切代码分支后，看情况是否要编译
- 如果要编译，输入命令 `build`，等待编译完成
- `tmux`
- `manual_mode`
- `roslaunch guiding_beacon_system guiding_station_main.launch`
- `ctrl+b` + `c` 新开 tmux 窗口
- 修改 `~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging/GS_measurement_3RB_aging_m3rb.py`
- 修改代码里面的 `GUESS_POSE`，这个是 `GS` 的预估位置
- 修改 `COLOR`
- 修改 `TEST_GOAL_0`，`TEST_GOAL_1`, `TEST_GOAL_2`里面的 `x`, `y`的数据，这是 3 个 RB 的位置数据
- 修改 `M_TEST_GOAL_0`，`M_TEST_GOAL_2`，`M_TEST_GOAL_2`里面的 `distance`，`rad_hor`，`rad_ver` 里面的值。 
- `cd ~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging`
- `python GS_measurement_3RB_aging.py`
- 观察 GS 会对两个 RB 做标定，并对三个 RB 作测量，总共三组测试
- 实验完成后，将`~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging` 下的 json 文件，复制到自己的电脑上
- `cd **/augustbot-tools/tools/scripts/tools/GS_aging` 进入自己电脑下的该文件夹，将 json 文件放置到该文件夹
- `python get_json_m3.py` 可以取得该次实验数据