# 发布流程

面向维护者。使用者的安装说明在仓库根 `README_zh.md`。

## 这个仓库不上传任何产物

一次发布 = **一个签名标签 + 一个 Gitea Release**。Release 不附产物，registry 也不上传。使用者的安装路径是 `git clone --branch v<版本>` 加 `colcon build`。

这是相对历史做法的**有意改变**，所以要说清理由。registry 里本仓曾有 `0.3.1 / 0.3.2 / 0.3.3 / 0.99.0 / 1.0.0 / 2.0.0` 共 6 个版本，`v1.0.0` 的 Release 也附了 wheel 与 sdist。停掉是因为那个产物是**惰性的** —— 已实测：

```
$ pip install --index-url <registry> linkerbot-hand-ros==2.0.0
成功，退出码 0
$ python -c "import linkerbot_hand_ros.node"
ModuleNotFoundError: No module named 'rclpy'
```

wheel 里也**根本没有 `linkerbot_hand_msgs`**（56 个条目，零个匹配）—— 它是 ROSIDL 接口，只能由 colcon 生成，没有平台无关 wheel。`setup.py` 同样不声明 `rclpy`，因为 ROS 侧依赖写在 `package.xml` 里。加上全仓文档零处让使用者 pip 装它、也找不到任何消费方，结论是：发一个跑不起来的包比不发更糟。

（这份文档更早的版本写过「本仓不发布到 registry」，随后又改成「一直在上传」。前者的结论对但依据不足，后者的事实对但把「历史这么做」当成了「应该继续这么做」。现在的位置是：历史确实上传过，但那个产物没有用途，所以停掉。）

## 为什么需要门禁

2.0.0 那次在本仓出了两类错，两类都被脚本拦住：

**标签打在了错误的提交上。** 当时文档修正 PR（#94）尚未合并，标签指向的提交里文档仍写着 1.0.0，只能删除远端标签重做。

**声明了装不上的依赖。** `setup.py` 写 `linkerbot-hand>=2.0.0,<3.0.0`，而 registry 里当时只有 `1.0.0`，照文档执行 `pip install` 直接得到 `No solution found`。版本号改对了，但没人检查这个范围是否真能解析 —— 这一条跨两个仓库，靠人对照必然会漏。

## 四个子命令

```bash
python3 scripts/release.py check                    # 只读，不改动任何东西
python3 scripts/release.py tag                      # check 通过后打签名标签、推送、建 Release
python3 scripts/release.py release                  # 只建 Release，用于标签已存在却缺 Release
python3 scripts/release.py verify-tag --tag v2.0.0  # CI 在标签推送时执行
```

### tag 一步做完，release 用于补建

`tag` 把打标签与建 Release 合成一步：中间没有需要构建的东西，而且此时 HEAD 就是被打标签的提交，所以从工作区生成的 Release 正文必然描述的就是标签指向的源码。

`release` 是补救路径，而且不是假想的 —— `v2.0.0` 就曾打了标签却没有 Release，只能手工补。它要求 `v<版本>` 存在**且正好指向 HEAD**：

```bash
git checkout v<版本>
python3 scripts/release.py release
```

不加这道门禁的话，正文会取自另一份源码的 CHANGELOG，Release 上写的就不是标签里的内容。

**两者都只在本地跑。** 门禁包含本仓的 pytest，而那需要 ROS jazzy 与 colcon 构建的消息；可用的 runner 只是普通 ubuntu 镜像，没有 ROS。在那上面加作业不会「排队等 runner」，而是一定失败。CI 只跑 `verify-tag`（不需要 ROS 工具链）。

需要 `packaging`（做版本范围比较）。查询 registry（用于依赖检查）与创建 Release 都需要 `GITEA_TOKEN`；registry 不可达时脚本明确报「未校验」而不是当作通过。

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
| 工作区 | 已跟踪文件有改动；不在 `main`，或与 `origin/main` 不一致（`release` 不校验后半项，它从标签的 detached checkout 运行） |
| CHANGELOG | 两个 `CHANGELOG.rst` 缺少本版本的 `<版本> (YYYY-MM-DD)` 章节；或 `Unreleased` 章节仍有内容 |
| 文档引用的范围 | 文档里写的 `linkerbot-hand>=…,<…` 与 `setup.py` 声明的不是同一个 |
| 文档钉住的版本 | `--branch v<版本>` 或 venv 目录名 `linkerbot-hand-ros-<版本>` 与本次发布不符；`linkerbot-hand==` 不满足 `setup.py` 的范围；各文档之间互相矛盾 |
| 文档里的散文版本 | 任何 `.md` 里出现了既不是本次发布、也不是所钉 hand/transport 版本的三段版本号。抓的是「验证应显示 hand `1.0.0`」这类没有 pin 模式可匹配的残留 |
| 依赖可安装 | 文档钉的 hand/transport 版本不在 registry 里；或 registry 中没有任何版本满足 `setup.py` 的范围 |
| 标签 | `tag` 时：`v<版本>` 已存在。`release` 时反过来：标签不存在，或不指向 HEAD |
| 质量 | `ruff check .` / `ruff format --check .` / `pytest` 未通过 |

历史版本引用应当写在 `CHANGELOG.rst` 里 —— 它不参与文档扫描。`.md` 文档里残留的旧版本是过期的操作指令，不是历史记录，所以一律拦下。

## Release 长什么样

标题 `linkerbot-hand-ros <版本>`。正文由两个 `CHANGELOG.rst` 的对应章节转换而来 —— 这些文件实际用到的两种 RST 写法（内联字面量与 `*` 条目）转成 Markdown，权威文本以链接给出 —— 再加安装说明与「为何不附产物」。**不附任何产物。** 已存在时跳过而非报错，所以重复执行无害。

## CI 做什么

`.gitea/workflows/release.yml` 在推送 `v*` 标签时执行 `release.py verify-tag`，跑的是**同一个脚本**，因此只有一套门禁。它校验标签名与该提交里声明的四处版本号一致、且文档钉的就是这个标签 —— 这是「标签打错提交」的事后拦截。

其余门禁需要 ROS 工具链，在打标签之前已经在本地跑过，CI 不重复；`tag` 会连带把 Release 建好。

Gitea runner 当前离线，工作流交付后只会排队不执行，现阶段本地脚本是真正生效的部分。

## 一次发布的完整顺序

1. 改四处版本号：`linkerbot_hand_ros/setup.py`、`linkerbot_hand_ros/package.xml`、`linkerbot_hand_msgs/package.xml`、`linkerbot_hand_ros/linkerbot_hand_ros/__init__.py`。
2. 两个 `CHANGELOG.rst` 的 `Unreleased` 内容移入带日期的版本章节，重建空的 `Unreleased`。
3. 更新 `setup.py` 的 `linkerbot-hand` 范围，以及文档里钉住的 `--branch v<版本>`、venv 目录名、`linkerbot-hand==`、`linkerbot-transport[all]==`。
4. **确认 hand 侧对应版本已经发布到 registry。** 否则第 3 步写的东西照抄下去装不上 —— 这正是 2.0.0 的故障。
5. 上述改动走正常的 Issue + PR 流程并合并。
6. `python3 scripts/release.py check` —— 确认全绿。
7. `python3 scripts/release.py tag` —— 打标签、推送、建 Release，一步完成。

第 5 步必须在第 7 步之前：标签指向当前提交，在文档修正合并之前打标签就会指向文档错误的提交，而 Release 正文取自那一刻的 CHANGELOG。
