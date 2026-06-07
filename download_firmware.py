#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的J-Link固件下载脚本
"""
import subprocess
import sys

hex_file = r"F:\ADC_FFT\Myproject\ADC_FFT_TEST\MDK-ARM\07-ADC_Test_LCD1_8\07-ADC_Test_LCD1_8.hex"
device = "STM32H750VBTx"
jlink_exe = r"C:\Program Files (x86)\SEGGER\JLink\JLink.exe"

# 创建J-Link脚本文件
script_content = f"""
h
connect
erase
loadfile "{hex_file}"
r
h
go
q
"""

script_file = r"F:\ADC_FFT\Myproject\ADC_FFT_TEST\download_script.jlink"
with open(script_file, 'w') as f:
    f.write(script_content)

# 构建J-Link命令
cmd = [
    jlink_exe,
    script_file
]

print(f"正在下载固件到 {device}...")
print(f"固件文件: {hex_file}")

try:
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    print(f"返回码: {result.returncode}")
    if result.stdout:
        print("标准输出:")
        print(result.stdout)
    if result.stderr:
        print("标准错误:")
        print(result.stderr)

    if result.returncode == 0:
        print("\n✅ 下载成功！")
        sys.exit(0)
    else:
        print(f"\n❌ 下载失败！返回码: {result.returncode}")
        sys.exit(1)
except subprocess.TimeoutExpired:
    print("❌ 下载超时！")
    sys.exit(1)
except Exception as e:
    print(f"❌ 错误: {e}")
    sys.exit(1)
