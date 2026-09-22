# TC264 智能车工程上下文

> 本文面向接手本仓库的人类或 AI。结论来自 2026-09-22 的源码快照；源码与本文冲突时，以源码为准。

> **AI 必读要求：** 任何 AI 读取本文并参与工程后，都必须在 `AI_WORK_LOG.md` 中简要记录自己的每次修改和每次尝试。记录必须包含时间、目标、涉及文件、实际操作、验证结果和遗留问题；失败、撤销、未产生代码改动的尝试也必须记录。开始工作前先阅读现有记录，结束回复中再向用户简要说明本轮修改和尝试。不得用笼统的“已优化”“已修复”代替可核对的事实。

## 1. 工程用途

这是一个基于逐飞科技 TC264 开源库开发的双核智能车工程，目标平台为 Infineon AURIX TC264D（工程配置中的处理器为 `tc26xb`、封装为 `bga292`）。车辆使用 MT9V03X 灰度摄像头识别赛道，通过双编码器闭环控制左右电机，并用舵机完成方向控制。

当前业务重点不是通用外设演示，而是：

- 摄像头图像二值化、边线提取和中线计算；
- 根据前瞻角控制舵机，根据弯道程度控制车速及左右轮差速；
- 识别并通过十字路口，包含近端/远端角点、状态机和虚拟补线；
- 通过 IPS200 屏幕显示巡线调试信息；
- 通过 WiFi SPI 向上位机发送图像、边线、角点和运行数据，并接收发车/调参命令；
- 出赛道保护、蜂鸣提示、里程和平均速度统计。

环岛类型已经在 `element.h` 中预留，但当前源码明确标注为尚未实现。

## 2. 开发和构建环境

- IDE/构建系统：AURIX Development Studio 的 Eclipse 工程；源码注释记录的开发环境为 ADS v1.10.2。
- 编译器/链接器：TASKING TriCore 工具链（报错前缀通常为 `ctc`、`ltc`、`amk`）。
- 工程配置：`.project`、`.cproject`。
- 链接脚本：`Lcf_Tasking_Tricore_Tc.lsl`。
- 构建产物：`Debug/`，目标文件名为 `Seekfree_TC264_Opensource_Library.elf`。
- 底层依赖：`libraries/` 中的逐飞驱动、Infineon iLLD 和公共组件。

不要假设普通 GCC/Clang 能直接构建本项目。代码使用 TASKING 的 section pragma、TriCore 中断宏和 AURIX 专用外设接口。

## 3. 双核运行架构

### CPU0：底层初始化和实时控制

入口为 `user/cpu0_main.c::core0_main()`。

CPU0 完成时钟、调试串口、屏幕、按键、IMU、编码器、舵机、电机、蜂鸣器和四路 PIT 定时器初始化。进入主循环后主要处理按键双击发车和上位机 `$GO` 发车命令。

所有已定义的 PIT、摄像头 ERU/DMA 和串口中断目前都使用 `IFX_INTERRUPT(..., 0, ...)`，即归 CPU0 执行。实时闭环因此实际运行在 CPU0 的中断上下文中。

### CPU1：视觉、元素识别和调试通信

入口为 `user/cpu1_main.c::core1_main()`。

CPU1 初始化 MT9V03X 摄像头和 WiFi SPI，然后在主循环中：

- 每 10 ms 扫描按键；
- 每 100 ms 刷新屏幕；
- 每 100 ms 发送运行数据；
- 停车时每 500 ms 轮询一次 WiFi 遥控命令；
- 收到新摄像头帧后执行完整图像处理；
- 最快每 33 ms 上传一帧二值图及边线/角点数据。

CPU0 和 CPU1 会共享图像结果、控制参数、发车标志和里程状态。修改这些全局量时必须检查并发可见性、原子性和中断竞态；不要因为当前代码可以编译就默认双核访问一定安全。

## 4. 主数据流

每个摄像头帧的主要调用顺序位于 `user/cpu1_main.c`：

```text
MT9V03X DMA 完成，mt9v03x_finish_flag 置位
  -> image_threshold(mt9v03x_image)  全局 Otsu 二值化
  -> find_edges_binary()             迷宫法提取左右边线
  -> process_edge_points()           逆透视、平滑、重采样、角度/NMS、角点和十字处理
  -> calculation_error()             计算前瞻中线与 pure_angle
  -> wifi_debug()                    按周期上传图像、边线、角点和补线
```

控制链路：

```text
pure_angle
  -> 2 ms 方向环 quadradic_pid_solve()
  -> angle
  -> servo_set()

angle + 边线长度
  -> 10 ms speed_control()
  -> 动态基础速度、内外轮差速
  -> 左右增量式 PID
  -> 电机 PWM
```

## 5. 定时任务和关键频率

| 周期 | 执行位置 | 工作 |
| --- | --- | --- |
| 2 ms | CPU0，CCU61 CH0 中断 | 方向 PID、舵机低通与回正限幅 |
| 5 ms | CPU0，CCU60 CH0 中断 | `system_ms` 计时、IMU 更新 |
| 10 ms | CPU0，CCU61 CH1 中断 | 编码器采样、里程统计、速度环、出赛道保护 |
| 10 ms | CPU1 主循环调度 | 按键扫描 |
| 33 ms | CPU1 主循环调度 | WiFi 图像/边线/角点上传上限约 30 fps |
| 100 ms | CPU1 主循环调度 | 屏幕刷新和 WiFi 数据上传 |
| 1 s | CPU0，CCU60 CH1 中断 | 帧率统计 |

上述周期是控制行为的一部分。增加耗时操作前，先确认它不在中断内，并评估 CPU1 图像处理和 WiFi 阻塞对帧率的影响。

## 6. 关键模块

| 文件/目录 | 作用 |
| --- | --- |
| `user/cpu0_main.c` | CPU0 入口、按键/WiFi 发车 |
| `user/cpu1_main.c` | CPU1 入口、视觉处理和调试任务调度 |
| `user/isr.c` | PIT、摄像头 DMA/ERU、串口等中断；实时控制核心 |
| `code/init.c` | 两个核的外设初始化编排 |
| `code/image.c/.h` | 二值化、迷宫巡线、点云处理、中线/转角、保护逻辑 |
| `code/element.c/.h` | 十字角点、轮廓爬线、十字状态机与虚拟补线；环岛仅预留 |
| `code/camera_param.c/.h` | 188 x 120 逆透视查找表 `invx`/`invy` |
| `code/pid.c/.h` | 舵机方向控制和电机增量式 PID |
| `code/motor.c/.h` | 编码器、电机 PWM、动态速度和差速控制 |
| `code/servo.h` | 舵机初始化/输出实现及标定参数；实现位于 `motor.c` |
| `code/data.c/.h` | 跨模块运行状态和可调参数的集中定义 |
| `code/display.c/.h` | IPS200 直接绘制、开机动画、调参 UI、按键发车 |
| `code/wifi_spi.c/.h` | WiFi 初始化、遥控命令解析、上位机调试协议 |
| `code/beep.c/.h` | 蜂鸣器输出 |
| `libraries/` | 逐飞外设库、Infineon iLLD、上位机协议组件 |
| `Lcf_Tasking_Tricore_Tc.lsl` | Flash、CPU0/CPU1 DSPR、栈、CSA 和 section 布局 |

仓库中包含一套 LVGL 8.4 源码和 `lvgl_demo.c`，但当前启动路径调用的是 `display.c::lcd_init()`，它直接初始化 IPS200；不要未经调用链确认就把 LVGL 当作当前 UI 主路径。

## 7. 重要状态、单位和约定

- 原始图像和鸟瞰图尺寸均为 188 x 120。
- `pixel_per_meter = 70.0f`，点云坐标常以米表示，显示/协议坐标常以像素表示。修改算法时要检查单位转换。
- `pure_angle` 是图像算法输出给方向环的前瞻角，单位为度。
- `angle` 是方向环输出的舵机目标角，最终受 `SMOTOR_LIMIT` 限制。
- 当前代码以负编码器目标表示前进；速度和电机符号不能按常见正向约定猜测。
- `encoder_measure_flag` 同时承担“正在行驶/测距”的重要门控作用；未发车时速度控制会保持电机硬停。
- `stop_flog`（原拼写保留）表示停车/保护状态。
- `total_distance` 是左右编码器绝对计数之和；标定值为 11485 count/m。
- `cross_flag` 的主状态为 `CR_NONE -> CR_START -> CR_ENTER -> CR_OUT -> CR_NONE`。
- `CROSS_ENABLE` 位于 `code/image.h`，当前为 1。
- `wifi_scratch_buf` 被 WiFi 二值图打包、轮廓坐标和爬线访问位图复用，依赖它们在 CPU1 同一主循环中顺序执行；不要并行调用。

### 7.1 当前十字状态机

- `CR_NONE -> CR_START`：必须在同一帧同时找到近端 `NL/NR` 和远端 `FL/FR` 四个角点；不允许单角点、边界接触等宽松条件触发。
- `CR_START -> CR_ENTER`：迷宫入口丢失、入口宽度大于 48 px，或近端双角点张开/移到 `y > 90`。`CR_START` 仍使用原普通寻线，不得用十字走廊接管转向。
- `CR_ENTER/CR_OUT`：只有这两个状态由十字动态走廊接管转向和显示。
- `CR_ENTER -> CR_NONE`：进入十字后至少行驶 0.44 m，且出口普通左右边线各恢复至少 25 个点、车头处赛道宽为 40~48 px，连续 3 帧成立后直接退回普通寻线。
- `CR_ENTER -> CR_OUT -> CR_NONE`：如果对面入口先回到图像底部，则进入 `CR_OUT`；普通双边线连续 3 帧恢复时退出，或者在 `CR_OUT` 行驶超过 0.17 m 强制退出。
- 状态显示必须互斥：`CR_NONE/CR_START` 只画普通边线与中线，`CR_ENTER/CR_OUT` 只画十字虚拟左右边线与中线。

### 7.2 上位机十字调试协议

- `$TMODE 0`：普通寻线模式，上位机图像右上角显示绿色标签。
- `$TMODE 1`：预十字，对应 `CR_START`，显示黄色标签。
- `$TMODE 2`：十字中，对应 `CR_ENTER/CR_OUT`，显示红色标签。
- 车端由 `code/wifi_spi.c` 每个图像调试周期发送状态；上位机源码为 `D:/code/codex/host-computer-main/host-computer-main/gnss_host.py`。运行打包后的 exe 时，必须重新打包才能带上新协议和标签。

常用控制参数集中在 `code/data.c`，包括 `servo_pid`、左右电机 PID、直道/弯道速度、弯道减速斜率、差速比例、内切参数和保护阈值。修改默认值时同时核对屏幕调参范围和 WiFi 协议。

## 8. 硬件和外部连接

已从源码确认的部分引脚/外设：

- 左电机 PWM：P33.6，方向：P33.7；
- 右电机 PWM：P02.4，方向：P02.5；
- 舵机 PWM：P33.9，频率 333 Hz；
- 蜂鸣器：P11.11；
- 左右编码器：GPT12 TIM6 和 TIM5；
- 摄像头：MT9V03X，188 x 120，VSYNC + DMA 采集；
- 显示：IPS200 SPI；
- 无线：逐飞 WiFi SPI 模块，TCP 客户端模式。

WiFi SSID、密码、服务器地址和端口目前直接写在 `code/wifi_spi.c`。更换网络环境时在该文件检查；文档不复制凭据。

## 9. 当前已知状态和风险

### 最近构建阻塞与已做处理

最近一次 Debug 构建报告：

```text
ltc E112: cannot locate 310 section(s)
requirement: 105K bytes of RAM area
range: 0x60000000-0x6001e000
group restriction: contiguous
```

该地址范围是 CPU1 的 120 KB DSPR1。链接脚本把默认 `.data/.bss` 作为一个 `ordered, contiguous` 组放到 DSPR1，同时 DSPR1 还要容纳 heap、用户栈、中断栈和 CSA，因此“总需求约 105 KB 小于 120 KB”也仍可能因保留区和连续区约束而失败。报错列出的若干 2/4 字节变量只是未能安置的 section 示例，不是根因本身。

直接原因是十字爬线新增的 `crawl_visited` 全帧位图占用 2820 字节常驻 RAM。当前源码已删除该静态数组，改为复用本帧已用完的 `rpts0an/rpts1an` 1600 字节和 `wifi_scratch_buf` 尾部 1220 字节，保留完整 188 x 120 位图且不修改 `image_binary`。容量已静态核算，但当前终端没有 TASKING `amk`，仍需在 AURIX Studio 重新链接确认 `ltc E112` 已消失。

如果仍报 RAM 错误，应先看最新 map 和 section 大小，重点检查图像缓冲、点云缓冲、WiFi scratch 和 LVGL 内存池；不要只通过缩小栈/CSA 掩盖问题。

### 其他注意事项

- 工作区可能存在尚未提交的算法修改。任何 AI 在编辑前都必须先执行 `git status --short` 和 `git diff -- <目标文件>`，不得覆盖其他协作者的改动。
- `camera_param.c` 是大型生成/标定数据文件，除非任务就是重做逆透视标定，否则不要手工格式化或重写。
- 图像处理、十字状态机、屏幕叠加和 WiFi 协议共享同一批角点/边线状态。修改数据结构时要搜索全部消费者。
- 中断函数中不能加入阻塞式 WiFi、屏幕输出或长循环。
- 一些旧源码注释存在编码乱码；修改局部逻辑时不要顺手转换整个文件编码，以免产生巨量无关 diff。

## 10. 建议的接手顺序

1. 先读 `user/cpu0_main.c`、`user/cpu1_main.c` 和 `user/isr.c`，建立双核与时序模型。
2. 视觉任务再读 `code/image.h`、`code/image.c`、`code/element.h`、`code/element.c`。
3. 控制任务再读 `code/data.c`、`code/pid.c`、`code/motor.c` 和 `code/servo.h`。
4. 调试链路再读 `code/display.c` 和 `code/wifi_spi.c`。
5. 涉及内存或链接错误时先读 `Lcf_Tasking_Tricore_Tc.lsl` 和最新 `.map`，不要凭变量名猜测。

## 11. 协作修改原则

- 先确认任务属于视觉、元素识别、控制、显示、通信还是链接布局，尽量限制修改范围。
- 先搜索变量的定义、写入点和全部读取点，特别是 CPU0 中断与 CPU1 主循环共享的变量。
- 保持实时路径可预测：中断只做有界、非阻塞工作。
- 算法改动至少记录输入单位、输出单位、状态切换条件和失败回退行为。
- 构建成功不代表实车安全；涉及 PWM、发车、保护或方向符号的修改必须说明需要台架/架空轮测试。
- 不提交 `Debug/` 生成物，也不要把本地 WiFi 凭据扩散到新文档或日志。

## 12. AI 修改与尝试记录要求

协作记录统一写入仓库根目录的 `AI_WORK_LOG.md`，按发生顺序追加，不覆盖其他协作者的内容。每次记录使用以下格式：

```markdown
## YYYY-MM-DD HH:mm - AI/会话标识

- 目标：本次要解决的问题。
- 修改：修改了哪些文件、符号或配置；没有修改则写“无”。
- 尝试：执行了哪些分析、命令或方案，包括失败和撤销的尝试。
- 验证：构建、测试、静态检查或实车验证的结果；未验证需说明原因。
- 遗留：尚未解决的问题、风险和下一步；没有则写“无”。
```

执行约束：

- 开始任务前先读 `PROJECT_CONTEXT.md` 和 `AI_WORK_LOG.md` 的最近记录。
- 每完成一次实际修改或一次尝试就补充记录；同一轮连续的小步骤可以合并，但不能遗漏失败方案或关键判断。
- 日志只记录事实和结果，不粘贴大量命令输出、二进制数据、密码或其他敏感信息。
- 不得修改或删除其他 AI 的历史记录；发现记录错误时追加更正说明。
- 最终回复必须简要概括本轮修改、尝试、验证结果和遗留问题，确保用户不打开日志也能了解进展。
