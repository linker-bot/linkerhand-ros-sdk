# 发布流程

面向维护者。使用者的安装说明在仓库根 `README_zh.md`。

## 这个仓库不上传任何产物

`linkerbot-hand-ros` **不发布到 registry**。全仓文档零处让使用者 `pip install linkerbot-hand-ros`；正式安装路径是 `git clone --branch v<版本>` 加 `colcon build`。而且即便 pip 装了也用不了：它靠 `data_files` 注册 ament 资源索引与 launch 文件，而 `linkerbot_hand_msgs` 是 ROSIDL 接口、没有平台无关 wheel，装进 venv 后缺消息类型，节点起不来。

所以这里的「发布」只有一件事：**打一个正确的标签。**

## 为什么需要门禁

2.0.0 那次在本仓出了两类错，两类都被脚本拦住：

**标签打在了错误的提交上。** 当时文档修正 PR（#94）尚未合并，标签指向的提交里文档仍写着 1.0.0，只能删除远端标签重做。

**声明了装不上的依赖。** `setup.py` 写 `linkerbot-hand>=2.0.0,<3.0.0`，而 registry 里当时只有 `1.0.0`，照文档执行 `pip install` 直接得到 `No solution found`。版本号改对了，但没人检查这个范围是否真能解析 —— 这一条跨两个仓库，靠人对照必然会漏。

## 三个子命令

```bash
python3 scripts/release.py check                    # 只读，不改动任何东西
python3 scripts/release.py tag                      # check 通过后打签名标签并推送
python3 scripts/release.py verify-tag --tag v2.0.0  # CI 在标签推送时执行
```

需要 `packaging`（做版本范围比较）。查询 registry 需要 `GITEA_TOKEN`；不可达时脚本明确报「未校验」而不是当作通过。

`check` 里的 `pytest` 用运行脚本的那个解释器执行，所以必须先备好 ROS 环境、并用能跑本仓测试的解释器调用脚本：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH=<hand 仓库>/src:$PYTHONPATH
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 <hand 仓库>/.venv/bin/python scripts/release.py check
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD` 脚本内部已设置，这里列出只为说明它是必需的 —— 本仓的 pytest 与 ROS jazzy 的 `launch_testing` 插件冲突（后者注册了过时的 `pytest_pycollect_makemodule` hookimpl）。`install/` 需要是最新构建：消息定义变更后不重新 `colcon build`，测试会以 `ImportError` 收集失败。

## 门禁清单

`check` 逐条校验，任一条失败即停且不产生副作用：

| 门禁 | 拦的是什么 |
| --- | --- |
| 版本一致 | `linkerbot_hand_ros/setup.py`、`linkerbot_hand_ros/package.xml`、`linkerbot_hand_msgs/package.xml`、`linkerbot_hand_ros/linkerbot_hand_ros/__init__.py` 四处版本号不一致。`linkerbot_hand_gui` 独立版本，不参与比对 |
| 工作区 | 已跟踪文件有改动；不在 `main`，或与 `origin/main` 不一致 |
| CHANGELOG | 两个 `CHANGELOG.rst` 缺少本版本的 `<版本> (YYYY-MM-DD)` 章节；或 `Unreleased` 章节仍有内容 |
| 文档引用的范围 | 文档里写的 `linkerbot-hand>=…,<…` 与 `setup.py` 声明的不是同一个 |
| 文档钉住的版本 | `--branch v<版本>` 或 venv 目录名 `linkerbot-hand-ros-<版本>` 与本次发布不符；`linkerbot-hand==` 不满足 `setup.py` 的范围；各文档之间互相矛盾 |
| 文档里的散文版本 | 任何 `.md` 里出现了既不是本次发布、也不是所钉 hand/transport 版本的三段版本号。抓的是「验证应显示 hand `1.0.0`」这类没有 pin 模式可匹配的残留 |
| 依赖可安装 | 文档钉的 hand/transport 版本不在 registry 里；或 registry 中没有任何版本满足 `setup.py` 的范围 |
| 标签 | `v<版本>` 在本地或远端已存在 |
| 质量 | `ruff check .` / `ruff format --check .` / `pytest` 未通过 |

历史版本引用应当写在 `CHANGELOG.rst` 里 —— 它不参与文档扫描。`.md` 文档里残留的旧版本是过期的操作指令，不是历史记录，所以一律拦下。

## CI 做什么

`.gitea/workflows/release.yml` 在推送 `v*` 标签时执行 `release.py verify-tag`，跑的是**同一个脚本**，因此只有一套门禁。它校验标签名与该提交里声明的四处版本号一致、且文档钉的就是这个标签 —— 这是「标签打错提交」的事后拦截。**不上传任何产物。**

其余门禁需要 ROS 工具链，在打标签之前已经在本地跑过，CI 不重复。

Gitea runner 当前离线，工作流交付后只会排队不执行，现阶段本地脚本是真正生效的部分。

## 一次发布的完整顺序

1. 改四处版本号：`linkerbot_hand_ros/setup.py`、`linkerbot_hand_ros/package.xml`、`linkerbot_hand_msgs/package.xml`、`linkerbot_hand_ros/linkerbot_hand_ros/__init__.py`。
2. 两个 `CHANGELOG.rst` 的 `Unreleased` 内容移入带日期的版本章节，重建空的 `Unreleased`。
3. 更新 `setup.py` 的 `linkerbot-hand` 范围，以及文档里钉住的 `--branch v<版本>`、venv 目录名、`linkerbot-hand==`、`linkerbot-transport[all]==`。
4. **确认 hand 侧对应版本已经发布到 registry。** 否则第 3 步写的东西照抄下去装不上 —— 这正是 2.0.0 的故障。
5. 上述改动走正常的 Issue + PR 流程并合并。
6. `python3 scripts/release.py check` —— 确认全绿。
7. `python3 scripts/release.py tag`

第 5 步必须在第 7 步之前：标签指向当前提交，在文档修正合并之前打标签就会指向文档错误的提交。
