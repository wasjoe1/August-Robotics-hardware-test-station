# DTU 老化

## DTU 老化服务器部署

1. 按照部署一台 `Lionel` 的流程部署一台树梅派
2. 按照 ./udev/ 配置 `udev`
3. 将 `DTU` 设备接入树梅派

---

## DTU 老化服务器的频道设置

1. 修文件夹下的 `dtu_config.py` 文件，将最后一行函数里面的参数修改成你想要的频道，我们默认设置为 `17`

    ```python
    set_channel(17)
    ```

2. 进入 `debug_shell`

    ```bash
    debug_shell
    ```

3. `DTU` 拨到配置模式，执行该脚本，脚本执行完后拨到正常模式

    ```bash
    python ~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging/script/dtu_config.py
    ```

---

## 准备设备

1. 待老化 `GS`
2. `RB` 3 个
3. `DTU` 老化服务器 `树梅派` `hostname:GSP4-0055`

---

## 准备工作

1. 将 `GS` 和 `RB` 放在指定位置，开机
2. `DTU`老化服务器开机接上
3. 将 `DTU` 老化服务器的 `DTU` 和 `GS` 的 `DTU` 设置为同一频道

---

## GS 开始老化

1. 打开 `GS` 的控制主页: http://gsp4-00xx.local
2. 手动标定 `GS`
3. 标定成功后，从主页上复制 `GS` 的位置，替换到 `gs_dtu_aging.py` 的 `GS` 的位置，保存退出

    ```python
    GUESS_POSE = (-0.0091,-0.2978,-1.550929)
    ```

4. 在该文件夹下执行该脚本

    ```bash
    cd ~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging
    python gs_dtu_aging.py
    ```

## DTU 老化服务器开始工作

1. 进入 `manual_mode`

    ```bash
    manual_mode
    ```

2. `tmux` 开一个窗口按顺序执行下列命令

    ```bash
    debug_shell
    roscore
    ```

3. `tmux` 开一个窗口按顺序执行下列命令

    ```bash
    debug_shell
    roslaunch boothbot_portal gs_hub.launch --screen
    ```

4. `tmux` 开一个窗口按顺序执行下列命令

    ```bash
    debug_shell
    vim ~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging/dtu_server_for_gs_aging.py
    ```

5. 编辑 `~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging/dtu_server_for_gs_aging.py` 里面的代码写入正确的位置信息

    ```python
    self.dis = [14.9,50,9]
    self.hor = [-1.51396,0.03396,1.1]
    ```

6. 执行该脚本开始老化

    ```bash
    cd ~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging
    python gs_dtu_aging.py
    ```
