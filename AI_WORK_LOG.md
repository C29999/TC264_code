# AI 修改与尝试记录

本文件用于多 AI 协作交接。所有参与工程的 AI 在开始工作前应先阅读 `PROJECT_CONTEXT.md` 和本文件的最近记录，并按 `PROJECT_CONTEXT.md` 第 12 节的格式追加工作记录。

禁止覆盖或删除已有记录。日志中不要写入 WiFi 密码、令牌或其他敏感信息。

## 2026-09-22 - Codex（当前会话）

- 目标：建立工程上下文，并要求后续 AI 简要描述和记录每次修改与尝试。
- 修改：新增 `PROJECT_CONTEXT.md`，记录工程用途、双核架构、数据流、关键模块、构建约束和当前 RAM 链接问题；随后在其开头和第 12 节加入强制记录规则，并新增本日志文件。
- 尝试：阅读 CPU0/CPU1 入口、中断、图像、十字识别、控制、显示、WiFi、工程配置、链接脚本和 map 文件，以源码交叉核对文档内容。
- 验证：检查 Markdown 为 UTF-8、内容完整且无替换字符；对文档执行 `git diff --check`。
- 遗留：尚未处理 TASKING 链接器 `ltc E112` 的 DSPR1 连续 RAM 布局失败；当前工作仅建立协作上下文和记录制度。

## 2026-09-22 01:05 - Codex（十字退出修复）

- 目标：修复车体已越过十字后仍停留在十字寻线状态的问题，并保持普通寻线和四角点入十字条件不变。
- 修改：在 `code/element.c` 增加出口普通双边线恢复判定：左右各至少 25 个点、车头处宽度 40~48 像素、连续 3 帧；`CR_ENTER` 在最短里程 0.44 m 后满足该条件会直接转为 `CR_NONE`，`CR_OUT` 也使用同一稳定判定。
- 尝试：阅读 `项目交接说明.md`、`PROJECT_CONTEXT.md`、`HANDOVER.md`、`AI_WORK_LOG.md`，并检查 CPU1 图像流程、十字状态机、编码器里程单位及普通边线数据的生命周期；确认原逻辑可能卡在只依赖 `maze_start_y > 93` 的 `CR_ENTER` 状态。
- 验证：`git diff --check` 通过；已核对里程单位为米，且状态更新发生在同帧 `calculation_error()` 之前。当前终端没有 TASKING `amk` 环境，未完成实际编译。
- 遗留：需在 AURIX Studio 重新构建，并用实车录像确认 25 点、40~48 像素和连续 3 帧对当前标定是否合适。

## 2026-09-22 01:06 - Codex（补记 RAM 压缩）

- 目标：解决新增十字轮廓访问位图导致的 DSPR1 链接超限。
- 修改：在 `code/element.c` 删除 2820 字节常驻 `crawl_visited`，将完整 188x120 位图分段复用本帧已用完的 `rpts0an/rpts1an` 和 `wifi_scratch_buf` 未被轮廓坐标占用的尾部；不修改 `image_binary`。
- 尝试：核对各缓冲的后续读取点和容量；位图需 2820 字节，两个 NMS 缓冲提供 1600 字节，WiFi 尾部需 1220 字节且可用 1420 字节。
- 验证：容量和最高索引静态核算通过，`git diff --check` 通过；当前终端无 TASKING `amk`，未完成实际链接。
- 遗留：需在 AURIX Studio 确认 `ltc E112` 已消失。

## 2026-09-22 01:10 - Codex（屏幕边界模式互斥）

- 目标：修复屏幕同时显示普通边线和十字补线两套边界的问题。
- 修改：`code/element.c::cross_build_vlines()` 改为只在 `CR_ENTER/CR_OUT` 生成十字走廊；`code/display.c::show_draw_edges()` 的十字角点、爬线轨迹和虚拟边线同样仅在 `CR_ENTER/CR_OUT` 绘制。`CR_NONE/CR_START` 只保留普通边线。
- 尝试：检查屏幕所有 `cross_flag`、`cross_line_active`、`cross_vline_valid` 绘制门控，确认 `cross_line_active` 在 `CR_START` 已置位，是两套边界同时出现的原因。
- 验证：已检查状态显示矩阵：`NONE/START` 普通边线，`ENTER/OUT` 十字走廊；`git diff --check` 通过，待 AURIX Studio 构建。
- 遗留：需用实车屏幕确认状态切换帧无残留。

## 2026-09-22 01:15 - Codex（上位机寻线状态标签）

- 目标：在上位机图像上明确显示“普通寻线模式”、“预十字”和“十字中”。
- 修改：`code/wifi_spi.c` 在每个图像调试周期发送 `$TMODE 0/1/2`，分别对应 `CR_NONE`、`CR_START`、`CR_ENTER/CR_OUT`；上位机 `gnss_host.py` 解析状态并在图像右上角以绿/黄/红标签显示中文模式名。
- 尝试：检查图像、boundary 和文本消息的发送顺序及上位机上一帧配对机制，选择独立数字状态协议，避免根据边线反推模式。
- 验证：上位机文件使用原有 UTF-8 BOM，改用 `utf-8-sig` 读取后 Python AST 语法检查通过；车端工程 `git diff --check` 通过。当前终端无 TASKING `amk`，未完成 AURIX 构建。
- 遗留：如果使用打包后的上位机 exe，需重新打包才能包含新标签。

## 2026-09-22 01:35 - Codex（交接文档同步）

- 目标：将本轮十字算法和调试协议的实际状态同步到工程主说明文档。
- 修改：更新 `PROJECT_CONTEXT.md`，记录四角点入十字条件、`CR_START/ENTER/OUT` 职责、出口双边线稳定判定、屏幕互斥显示、`$TMODE` 映射、上位机源码位置及 2820 字节 RAM 复用方案。
- 尝试：检查根目录所有 Markdown；`PROJECT_CONTEXT.md` 是唯一有内容的主交接文档，`项目交接说明.md` 和 `HANDOVER.md` 为 0 字节占位，因此未制造三份重复且可能失同的说明。
- 验证：已检查新增章节和关键字，`git diff --check` 通过。
- 遗留：十字参数仍需 AURIX Studio 构建和实车验证。

## 2026-09-22 - Codex（STC32 十字行为整体切换）

- 目标：用只读参考工程 `stc32-main/project/code/element.c::element_cross` 和 `pure_track.c` 的生效逻辑取代旧的“四角点严格进入、START 不接管、退出连续 3 帧”规格。
- 修改：`code/element.c` 按参考 `element.c:107` 改为双近端角点/左角点+右触界/右角点+左触界三选一；按 `:434-465` 扩展 START 扫描并在 ENTER 清行锁存；按 `:597-624` 改为 ENTER 超时直接 NONE、OUT 第 5 点宽度单帧退出。`code/image.c/.h` 按 `pure_track.c:70-76,120-150,174-184,203-216` 增加 START 左角点优先的截断单边中线、半宽偏移、pure pursuit 和角度跳变门控，且不改原点集。`code/display.c` 和 `code/wifi_spi.c` 改为 START 显示/发送同一条截断中线；`$TMODE` 保持不变。`PROJECT_CONTEXT.md` 已同步新规格。
- 尝试：开工前执行 `git status`、`git log --oneline -5`、`git diff`，确认并保留 `code/data.c` 的 `kp=1.5`、直道速度 `-220` 未提交调参；阅读并对照 STC32 指定行号；全量搜索 `cross_flag`、`cross_line_active`、`cross_vline_valid`、`touch_boundary0/1`、`far_Lpt0/1`的生产者与消费者。远端爬线代码保留但已从流水线停用。
- 验证：修改文件的花括号/圆括号计数匹配，`git diff --check` 无空白错误。本机无 TASKING `amk`，未编译；需在 AURIX Studio 重点确认停用 `find_far_corners_crawl()` 后的未使用函数/变量告警和 RAM 链接。
- 遗留：未做实车验证；`CR_START_DANGLE = 5° / SMOTOR_RATE` 是按本车角度量纲换算的初值，需实车标定。

## 2026-09-23 - Codex（WiFi 图像切回二值）

- 目标：逆透视标定完成后，让上位机接收当前寻线使用的二值图，而不是灰度相机原图。
- 修改：`code/wifi_spi.c::wifi_image_send()` 将 `image_binary` 按逐飞协议 MSB-first 打包为 1 bit/像素，类型切回 `SEEKFREE_ASSISTANT_CAMERA_TYPE_BINARY`；复用已有 `wifi_scratch_buf`，没有增加整帧常驻内存。同步修正 `code/image.h` 的缓冲用途注释。
- 尝试：核对逐飞组件中二值图每 8 像素/字节的长度计算，以及外部上位机 `gnss_host.py` 对 type 1 图像的 MSB-first 解包；确认 `MT9V03X_W * MT9V03X_H` 可整除 8。
- 验证：`git diff --check` 通过；本次未编译、未连接实车验证。
- 遗留：在上位机确认收到的图像标签为 `BIN`，并检查边线叠加与实际二值画面配准。

## 2026-09-23 - Codex（全丢线诊断显示）

- 目标：推车复现全丢线时，区分二值起点搜索失败与逐行赛道跟踪失败。
- 修改：`code/display.c` 主调试页增加 `RAW`（左右原始边界点数）、`SY`（起始行）、`ENTRY`（起始行左右坐标）和 `T`（左右触界标志）；保留现有 `pts`、`ST` 和 `CR` 显示，不改寻线/十字算法。
- 尝试：对照 `find_binary_start()`、`track_lane_by_rows()`、`find_edges_binary()` 和 `calculation_error()` 的状态更新顺序，选择可以区分入口失败与后续取点/控制失败的现场参数。
- 验证：待执行 `git diff --check`；本次未编译、未实车测试。
- 遗留：请推车复现并拍下主屏 `RAW/SY/ENTRY/T/pts/ST/CR` 数值；`SY=-01` 代表没找到起始白色区间，`SY>=0` 但 `RAW` 点数少代表后续逐行跟踪提前中断。

## 2026-09-23 - Codex（普通边线贴边容错）

- 目标：优化原图二值化上的普通巡线，避免白色车道贴近相机边缘时被误判全丢线。
- 修改：`code/image.c` 的 `find_binary_start()` 和 `track_lane_by_rows()` 允许白色赛道段一侧贴图像边界；仍保留最小/最大宽度和逐行中心跳变限制，且触界标志继续记录给十字判据使用。
- 约束：未改变逆透视用途、十字状态机或普通巡线的数据源；普通寻线仍只读取 `image_binary`。
- 验证：待执行 `git diff --check`；本次未编译、未实车验证。
- 遗留：需实车确认弯道/十字入口不会因贴边容错增加误触发。

## 2026-09-23 - Codex（WiFi 发送普通巡线起始行诊断）

- 目标：在上位机同步查看车端自适应起始行是否找到，以及逐行跟踪实际输出点数。
- 修改：`code/wifi_spi.c` 新增 `$TRACK sy entry_l entry_r raw_l raw_r touch_l touch_r`；保留原 `$DBG fps error` 不变。上位机 `gnss_host.py` 解析该行，并在图像左上角第二行显示 `sy/entry/raw/t`。
- 验证：待执行 `git diff --check`；本次未编译、未实车验证。
- 遗留：烧录后观察 `sy=-1`（起始行失败）或 `sy>=0、raw 点数偏少`（逐行跟踪失败）。

## 2026-09-24 - Codex（十字恢复原图二值四角点方案）

- 目标：撤销十字对逆透视图的依赖，并参考用户提供的 CSDN 四角点/斜率补线代码降低弯道误判。
- 修改：`code/element.c` 删除 `cross_ipm_lane_restored()` 及全部 IPM 出口判据；十字中心扫描继续直接读取 `image_binary`。`CR_NONE` 保留本帧爬线得到的远端角点，仅在左右巡线同时触界且 NL/NR/FL/FR 四角点齐全时进入 `CR_START`；两条补边线和中点连线仍独立锁存，不覆盖普通边界数组。`CR_ENTER/OUT` 恢复原图边线点数、第 5 点宽度连续 3 帧与编码器超时退出。`code/image.c` 恢复起始行必须同时存在左右真实黑白跳变，起始后的逐行跟踪仍允许贴边。
- 参考：用户粘贴的 `Cross_Detect()` 以双边丢线为前置、寻找左右上下角点，并按四点/三点/两点组合连线或延长边界；本工程采用更严格的双触界+四点齐全进入，降低普通弯道短暂丢线误判。
- 验证：`git diff --check` 通过；静态搜索确认 `code/element.c` 已无 `img_pers_data`、`cross_ipm` 或 `CR_IPM` 消费。本机未编译、未实车验证。
- 遗留：需在 AURIX Studio 构建，重点确认 TASKING 未使用宏/静态函数告警及 RAM 链接；实车验证四角点检出率、0.44/1.3/0.17 m 状态距离和 40~48 px 出口宽度。

## 2026-09-24 - Codex（按参考代码实现分情况补线）

- 目标：按用户粘贴的 `Cross_Detect()` 四种角点组合补线，而不是只允许四点齐全时直连。
- 修改：`code/element.c::cross_build_vlines()` 左右侧独立处理：真实远端角点存在时直接连接；缺失时用近端角点前第 7 点到前第 2 点估算边界切线，并向上延长至 y=10。两侧结果取中点生成虚拟中线，覆盖四点、任一侧缺远端点、两侧都缺远端点四种情况。
- 约束：不写回 `left_line_points/right_line_points` 或 `rpts0s/rpts1s`；状态机仍要求双边触界、左右近端角点和两侧有效补线同时成立，防止普通弯道单角点误判。
- 验证：待执行 `git diff --check` 和静态括号检查；本机未编译、未实车验证。
- 遗留：实车观察延长线是否贴合边界；如透视导致直线外推偏差，再标定取样点跨度和目标 y。
### 2026-09-24 十字补线整数化
- 十字确认后新增 `cross_apply_integer_edges()`：按图像逐行计算整数 x，直接写回 `left_line_points/right_line_points`，并同步当前 `rpts0s/rpts1s`，避免浮点直线取整造成重复坐标和补线截断。
- `cross_extend_far()` 与回写函数增加斜率绝对值范围 `0.35..8.0`，不满足范围时放弃该侧补线，阈值待实车标定。
- 未使用 TASKING/AURIX 编译器，未完成编译和实车验证；需在 AURIX Studio 检查链接、告警及补线方向。
### 2026-09-24 十字速度环停车
- 十字状态从 `CR_START` 起将速度环目标设为 0，通过编码器速度 PID 减速停车，不调用硬停分支。
- 原 `cross_speed` 保留但不再用于十字停车目标；退出十字后恢复普通动态速度。
