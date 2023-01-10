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

1. 打开 `GS` 的控制主页: http://gspx-0xxx.local
2. 将 GS 设备设置成 `DTU` 工作模式，然后进入 docker_mode，请注意不要有其他设备工作在同一个 `DTU` 频道下

    ```bash
    docker_mode
    ```

3. 手动标定 `GS`，然后通过日志得到 `15米` `RB` 和 `48米` `RB` 的水平弧度值

    ```bash
    roslog meas
    ```

    进入日志后，依次使用下面命令，可看到页面上显示角度值，将 `15米` `RB` 和  `48米` `RB` 的值记下来

    ```bash
    /
    at 
    ```

4. 标定成功后，从主页上复制 `GS` 的位置，替换到 `gs_dtu_aging.py` 的 `GS` 的位置，修改 `RB` 的颜色，保存退出

    ```python
    COLOR = "ROG"
    GUESS_POSE = (-0.0091, -0.2978, -1.550929)
    ```

5. 在该文件夹下执行该脚本

    ```bash
    cd ~/catkin_ws/src/augustbot-tools/tools/scripts/tools/GS_aging
    python gs_dtu_aging.py
    ```

## DTU 老化服务器开始工作

1. 进入 `DTU` 老化服务器 `cali_mode`

    ```bash
    ssh augbooth@gsp4-0055.local
    ```

    ```bash
    cali_mode
    ```

2. `tmux` 开一个窗口按顺序执行下列命令，设置 `RB` 的颜色以及三个 `RB` 的水平角度值，`0_` 和 `1_` 分别是  `15米` `RB` 和  `48米` `RB` 的角度值  

    ```bash
    debug_shell
    rostopic pub /set_rb std_msgs/String "data: 'c_ROG'"
    rostopic pub /set_rb std_msgs/String "data: '0_-1.55029'"
    rostopic pub /set_rb std_msgs/String "data: '1_-0.02354'"
    rostopic pub /set_rb std_msgs/String "data: '2_1.1'"

    ```
