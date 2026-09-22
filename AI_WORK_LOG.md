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
