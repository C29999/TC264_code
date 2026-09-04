#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
解析春之雪上位机记录的智能车原始数据，在图像上绘制赛道边线。

数据格式：
  [XXXXX] t=XXXs channel=CAR len=XXXX
  HEX: AA 6E ... (左边界数据) AA 95 ... (右边界数据)

左边界帧头: AA 6E 03 01 C8 00 00 F8, 后跟 120 个 (left_col, 0x00) 对
右边界帧头: AA 95 03 01 C8 00 1F 00, 后跟 120 个 (right_col, 0x00) 对
"""

import re
import sys
import os
from pathlib import Path

# 默认数据文件路径
DEFAULT_DATA = r"C:\Users\10503\Documents\Codex\2026-08-05\d-code-arm\gnss_host\recorded_data\smartcar_raw_20260904_134253.txt"

# 图像尺寸 (MT9V03X)
MT9V03X_W = 188
MT9V03X_H = 120


def parse_hex_line(hex_text):
    """解析 HEX 行，返回字节列表。"""
    hex_text = hex_text.strip()
    if hex_text.startswith("HEX:"):
        hex_text = hex_text[4:].strip()
    # 移除可能的 ASCII 部分（在 HEX 行末尾可能混入的普通文本）
    # 只保留有效的十六进制对
    parts = hex_text.split()
    bytes_list = []
    for p in parts:
        try:
            if len(p) == 2:
                bytes_list.append(int(p, 16))
        except ValueError:
            continue
    return bytes_list


def parse_data_file(filepath):
    """解析数据文件，返回帧列表。每帧包含 left_edge 和 right_edge 列表。"""
    frames = []
    current_hex_buffer = bytearray()
    
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    print(f"总行数: {len(lines)}")
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # 跳过空行
        if not line:
            i += 1
            continue
        
        # 查找数据块开始: [XXXXX] t=XXXs channel=CAR len=XXXX
        block_match = re.match(r'^\[(\d+)\]\s+t=([\d.]+)s\s+channel=(\w+)\s+len=(\d+)', line)
        if block_match:
            block_num = int(block_match.group(1))
            timestamp = float(block_match.group(2))
            data_len = int(block_match.group(4))
            
            # 下一行应该是 HEX 行
            i += 1
            while i < len(lines):
                next_line = lines[i].strip()
                if next_line.startswith("HEX:"):
                    hex_line = next_line
                    hex_bytes = parse_hex_line(hex_line)
                    current_hex_buffer.extend(hex_bytes)
                    i += 1
                elif next_line.startswith("ASCII:"):
                    i += 1
                elif not next_line:
                    i += 1
                else:
                    break
            
            if block_num == 1:
                print(f"Block 1: buffer len={len(current_hex_buffer)}")
                if len(current_hex_buffer) > 20:
                    print(f"  First 20 bytes: {' '.join(f'{b:02X}' for b in current_hex_buffer[:20])}")
                    # Look for AA 6E and AA 95
                    for idx in range(len(current_hex_buffer) - 1):
                        if current_hex_buffer[idx] == 0xAA and current_hex_buffer[idx+1] == 0x6E:
                            print(f"  Found AA 6E at offset {idx}")
                            print(f"    Next 16 bytes: {' '.join(f'{b:02X}' for b in current_hex_buffer[idx:idx+16])}")
                        if current_hex_buffer[idx] == 0xAA and current_hex_buffer[idx+1] == 0x95:
                            print(f"  Found AA 95 at offset {idx}")
                            print(f"    Next 16 bytes: {' '.join(f'{b:02X}' for b in current_hex_buffer[idx:idx+16])}")
            
            # 解析当前缓冲区数据
            frame = parse_frame_data(bytes(current_hex_buffer), block_num, timestamp)
            if frame:
                frames.append(frame)
            
            # 清空缓冲区，准备下一帧
            current_hex_buffer = bytearray()
            continue
        
        i += 1
    
    return frames


def parse_frame_data(hex_bytes, block_num, timestamp):
    """从字节数据中提取左边界和右边界。"""
    result = {"block": block_num, "time": timestamp, "left": None, "right": None}
    
    # 查找左边界帧头 AA 6E
    left_start = find_pattern(hex_bytes, [0xAA, 0x6E])
    # 查找右边界帧头 AA 95
    right_start = find_pattern(hex_bytes, [0xAA, 0x95])
    
    if left_start is not None:
        # 帧头 8 字节后跟 120 个 (col, 0x00) 对
        data_start = left_start + 8
        left_edge = []
        for j in range(MT9V03X_H):
            pos = data_start + j * 2
            if pos + 1 < len(hex_bytes):
                col = hex_bytes[pos]
                left_edge.append(col)
        if len(left_edge) == MT9V03X_H:
            result["left"] = left_edge
    
    if right_start is not None:
        data_start = right_start + 8
        right_edge = []
        for j in range(MT9V03X_H):
            pos = data_start + j * 2
            if pos + 1 < len(hex_bytes):
                col = hex_bytes[pos]
                right_edge.append(col)
        if len(right_edge) == MT9V03X_H:
            result["right"] = right_edge
    
    return result


def find_pattern(data, pattern):
    """在字节列表中查找指定模式，返回起始索引。"""
    pat = bytes(pattern) if isinstance(pattern, list) else pattern
    for i in range(len(data) - len(pat)):
        if data[i:i+len(pat)] == pat:
            return i
    return None


def create_edges_image(left_edge, right_edge, width=MT9V03X_W, height=MT9V03X_H):
    """
    根据边线数据创建图像。
    返回一个 2D 列表 (height x width)，0=黑色, 255=白色。
    """
    # 创建黑色背景
    image = [[0] * width for _ in range(height)]
    
    for y in range(height):
        left = left_edge[y] if left_edge and y < len(left_edge) else None
        right = right_edge[y] if right_edge and y < len(right_edge) else None
        
        if left is not None and right is not None and left < right:
            # 赛道区域填充白色
            for x in range(left, min(right + 1, width)):
                image[y][x] = 255
    
    return image


def render_edges_with_matplotlib(left_edge, right_edge, block_num, timestamp, width=MT9V03X_W, height=MT9V03X_H):
    """使用 matplotlib 绘制边线图。"""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("需要安装 matplotlib: pip install matplotlib")
        return None
    
    # 创建图像
    img = create_edges_image(left_edge, right_edge, width, height)
    img_array = np.array(img, dtype=np.uint8)
    
    # 绘制
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))
    
    # 显示图像
    ax.imshow(img_array, cmap='gray', vmin=0, vmax=255, aspect='auto')
    
    # 绘制边线
    ys = np.arange(height)
    
    if left_edge:
        valid_left = [(y, left_edge[y]) for y in range(height) if 0 <= left_edge[y] < width]
        if valid_left:
            ly, lx = zip(*valid_left)
            ax.plot(lx, ly, 'r-', linewidth=2, label='左边界')
    
    if right_edge:
        valid_right = [(y, right_edge[y]) for y in range(height) if 0 <= right_edge[y] < width]
        if valid_right:
            ry, rx = zip(*valid_right)
            ax.plot(rx, ry, 'g-', linewidth=2, label='右边界')
    
    # 中线
    mid_x = width / 2
    ax.axvline(x=mid_x, color='y', linestyle='--', linewidth=1, alpha=0.5, label='中线')
    
    ax.set_title(f'数据块 #{block_num}  t={timestamp:.3f}s')
    ax.set_xlabel('列 (X)')
    ax.set_ylabel('行 (Y)')
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)  # Y 轴翻转，顶部为 row 0
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 添加坐标标注
    for y in range(0, height, 20):
        if left_edge and y < len(left_edge):
            ax.annotate(f'L={left_edge[y]}', (left_edge[y], y), 
                       fontsize=6, color='red', alpha=0.7)
        if right_edge and y < len(right_edge):
            ax.annotate(f'R={right_edge[y]}', (right_edge[y], y),
                       fontsize=6, color='green', alpha=0.7)
    
    plt.tight_layout()
    return fig


def main():
    # 检查命令行参数
    filepath = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DATA
    
    if not os.path.exists(filepath):
        print(f"文件不存在: {filepath}")
        return
    
    print(f"解析数据文件: {filepath}")
    frames = parse_data_file(filepath)
    print(f"共解析到 {len(frames)} 帧数据")
    
    # 显示统计信息
    valid_frames = [f for f in frames if f["left"] is not None or f["right"] is not None]
    print(f"包含边线数据的帧: {len(valid_frames)}")
    
    if valid_frames:
        print(f"\n第一帧数据块 #{valid_frames[0]['block']}:")
        if valid_frames[0]["left"]:
            left = valid_frames[0]["left"]
            print(f"  左边界: 行0={left[0]}, 行59={left[59]}, 行119={left[119]}")
        if valid_frames[0]["right"]:
            right = valid_frames[0]["right"]
            print(f"  右边界: 行0={right[0]}, 行59={right[59]}, 行119={right[119]}")
    
    # 尝试用 matplotlib 绘图
    if valid_frames:
        try:
            fig = render_edges_with_matplotlib(
                valid_frames[0]["left"], 
                valid_frames[0]["right"],
                valid_frames[0]["block"],
                valid_frames[0]["time"]
            )
            if fig:
                # 保存图片
                output_dir = Path(filepath).parent / "output"
                output_dir.mkdir(exist_ok=True)
                output_path = output_dir / f"frame_{valid_frames[0]['block']:05d}.png"
                fig.savefig(str(output_path), dpi=150)
                print(f"\n图像已保存: {output_path}")
                
                # 显示
                plt.show()
        except ImportError:
            print("\nmatplotlib 未安装，跳过绘图。")
            print("安装: pip install matplotlib")
        
        # 输出所有帧的边线数据为 CSV（方便其他工具使用）
        output_csv = Path(filepath).parent / "output" / "edges_data.csv"
        output_csv.parent.mkdir(exist_ok=True)
        with open(output_csv, 'w', encoding='utf-8') as f:
            f.write("block,time,row,left_col,right_col\n")
            for frame in valid_frames:
                for y in range(MT9V03X_H):
                    left = frame["left"][y] if frame["left"] and y < len(frame["left"]) else -1
                    right = frame["right"][y] if frame["right"] and y < len(frame["right"]) else -1
                    f.write(f"{frame['block']},{frame['time']:.3f},{y},{left},{right}\n")
        print(f"边线数据已导出: {output_csv}")


if __name__ == "__main__":
    main()