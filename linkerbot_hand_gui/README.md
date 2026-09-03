# linkerbot_hand_gui

O6 的 ROS 2 URDF 关节滑块预览和实机控制桥。这个 package 不直接打开
CAN，也不创建第二个 `HandDevice`；所有设备 I/O 仍由 `linkerbot_hand_ros`
和 `linkerbot-hand` 负责。

## 资源准备

O6 的 URDF、decoder 和 mesh 来自
[`linker-sim-assets`](https://github.com/linker-bot/linker-sim-assets)。由于
mesh 使用 Git LFS，建议先安装 Git LFS 并完整检出资源仓库，然后设置资源根目录：

```bash
git lfs install
git clone https://github.com/linker-bot/linker-sim-assets.git ~/linker-sim-assets
cd ~/linker-sim-assets
git lfs pull
export LINKERBOT_HAND_ASSETS_ROOT="$PWD/src/linker_robot_assets/assets"
```

如果没有安装 Git LFS，mesh 文件会保留为 LFS 指针，无法显示完整外观；请使用
`git lfs pull` 获取真实 mesh。GUI 不复制这些二进制资源。纯预览会在发现任一
visual GLB 不完整时直接报错，不会用无材质的 OBJ 拼出颜色不一致的模型。
O6 右手拇指关节的官方 GLB 有一个缺少颜色参数的材质槽；GUI 会在用户缓存目录
生成补全为同系列关节灰色的 GLB 副本，既保证交界件颜色一致，也不会修改外部资源仓库。

也可以安装包含资源的 `linker_robot_assets` Python 包；启动器会自动调用
`linker_robot_assets.asset_root()`。显式的 `assets_root` launch 参数优先级最高。

## 启动

### 仅预览 URDF

如果只需要查看模型和拖动关节，不连接设备也不加载 SDK，使用
`preview_only:=true`。该模式只启动 URDF、滑块和 RViz，不创建 bridge 或任何
`/linkerbot_hand/command/*` 发布器：

```bash
source /opt/ros/jazzy/setup.bash
source ~/code/linkerbot-hand-ros/install/setup.bash
export LINKERBOT_HAND_ASSETS_ROOT=~/linker-sim-assets/src/linker_robot_assets/assets
export QT_QPA_PLATFORM=xcb

ros2 launch linkerbot_hand_gui hand_gui.launch.py \
  side:=right \
  assets_root:="$LINKERBOT_HAND_ASSETS_ROOT" \
  preview_only:=true
```

滑块输出为 `/linkerbot_hand_gui/preview_joint_states`，仅用于本地模型预览。
它不会向 O6 实物发送消息，也不会提供 `set_control_enabled` 服务。

先启动 O6 驱动，再启动 GUI（一次命令同时打开 URDF 渲染和滑块窗口）：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=o6 side:=right transport_json:='{"backend":"socketcan","channel":"can1"}'

ros2 launch linkerbot_hand_gui hand_gui.launch.py \
  side:=right \
  assets_root:="$LINKERBOT_HAND_ASSETS_ROOT" \
  driver_name:=/linkerbot_hand
```

启动后会出现独立的 `joint_state_publisher_gui` 滑块窗口（不是 RViz 内嵌面板）。启动器同时建立 `world` 到
实际/目标两个模型根节点的静态 TF。滑块发布器会持续发布当前值，bridge 在启用
控制后按 `command_rate_hz` 发送最新目标；停止或重新启用控制都会清空目标。
默认 launch 已经自动启动 RViz2；如需单独打开 RViz2，可用以下命令：

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix linkerbot_hand_gui)/share/linkerbot_hand_gui/config/o6.rviz"
```

如果只看到空白 RViz，请检查终端是否报告 mesh 加载错误。资源仓库需要完整的
Git LFS 文件；当前 RViz（通过 Assimp）支持 `.glb`，预览模式不会把 `.obj` 作为
visual 降级资源，以避免手掌和指节材质不一致：

```bash
git lfs install
git -C ~/linker-sim-assets lfs pull
```

滑块窗口若位于另一块屏幕，可用桌面窗口管理器将标题为 `Joint State Publisher`
的窗口移回主屏；启动器已强制使用 Qt XCB 后端以避免 Wayland 下窗口不可见。

默认是“仅预览”，滑块只驱动 `target/` TF，不发送实机命令。确认安全并且
驱动已经发布 `identity.verified=true` 后，显式启用控制：

```bash
ros2 service call /o6_gui_bridge/set_control_enabled std_srvs/srv/SetBool \
  "{data: true}"
```

启用后再次拖动滑块，bridge 以固定频率发布最新目标到
`/linkerbot_hand/command/angles`。中间只保留一个目标，不建立历史队列。停用控制
会清空未发送目标，重新启用后必须再次移动滑块。

## 主题和安全边界

| 主题 | 方向 | 说明 |
|---|---|---|
| `/linkerbot_hand_gui/raw_target_joint_states` | GUI 内部 | 滑块原始 URDF 弧度目标 |
| `/linkerbot_hand_gui/target_joint_states` | 输出 | 目标模型 TF 输入 |
| `/linkerbot_hand_gui/actual_joint_states` | 输出 | `state/angles` 的实际反馈 TF 输入 |
| `<driver>/capabilities` | 订阅 | 运行时型号、左右手和关节顺序 |
| `<driver>/state/angles` | 订阅 | 实际 percent 状态 |
| `<driver>/command/angles` | 发布 | 仅在显式启用且状态新鲜时发布 percent 命令 |

Bridge 会校验 O6 型号、左右手、六个 capability 关节名称和顺序。URDF 的限位只
用于几何显示和滑块到 percent 的换算；真实命令范围和设备能力仍以 SDK capability
为准。`physical_rad` 不可用时，反馈模型是基于 percent 与 URDF 限位的几何映射，
不是伪造的实测物理角度。

## 当前限制

- 当前版本只支持 O6；其他型号的 URDF 不会被接受。
- 不实现任意 link/指尖拖动和 IK。
- 需要 `linker-sim-assets` 的 Git LFS visual mesh 才能在 RViz 显示完整外观；纯预览
  会在资源不完整时拒绝启动并提示缺失文件。请先执行 `git lfs pull`，不要用部分
  下载的指针文件替代 GLB。
