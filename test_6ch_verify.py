#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
6通道ADC测试验证脚本
用于验证所有通道正常触发、时间准确、频率准确
"""

import serial
import time
import re
import threading
import queue

class ADCTestMonitor:
    def __init__(self, port='COM14', baudrate=2000000):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.event_queue = queue.Queue()
        self.stats = {
            'events': [],
            'dedupped': [],
            'test_cases': [],
            'compare_data': []
        }

    def open(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1
            )
            print(f"✅ 串口 {self.port} 已打开")
            return True
        except Exception as e:
            print(f"❌ 串口打开失败: {e}")
            return False

    def close(self):
        if self.ser:
            self.ser.close()
            print(f"✅ 串口 {self.port} 已关闭")

    def read_thread(self):
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.process_line(line)
            except Exception as e:
                if self.running:
                    print(f"读取错误: {e}")

    def process_line(self, line):
        # 解析EVENT
        match = re.search(r'>>> EVENT (\S+) T=(\d+)s(\d+)ms(\d+)us freq=([\d.]+)', line)
        if match:
            event = {
                'type': 'event',
                'channel': match.group(1),
                'time_s': int(match.group(2)),
                'time_ms': int(match.group(3)),
                'time_us': int(match.group(4)),
                'freq': float(match.group(5)),
                'raw': line
            }
            self.stats['events'].append(event)
            self.event_queue.put(event)
            return

        # 解析DEDUP_DROP
        match = re.search(r'\[DEDUP_DROP\] (\S+) F#(\d+): dt=(\d+)ms', line)
        if match:
            drop = {
                'type': 'dedup',
                'channel': match.group(1),
                'frame': int(match.group(2)),
                'dt_ms': int(match.group(3)),
                'raw': line
            }
            self.stats['dedupped'].append(drop)
            return

        # 解析TEST CASE
        match = re.search(r'\[TEST CASE\] (Starting|Finished): (.+)', line)
        if match:
            tc = {
                'type': 'test_case',
                'action': match.group(1),
                'name': match.group(2),
                'raw': line
            }
            self.stats['test_cases'].append(tc)
            return

        # 解析COMPARE
        if '[COMPARE]' in line:
            comp = {
                'type': 'compare',
                'data': line,
                'raw': line
            }
            self.stats['compare_data'].append(comp)
            return

    def start_monitor(self):
        self.running = True
        self.thread = threading.Thread(target=self.read_thread, daemon=True)
        self.thread.start()
        print("✅ 监控线程已启动")

    def stop_monitor(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=2)
        print("✅ 监控线程已停止")

    def print_summary(self):
        print("\n" + "="*80)
        print("6通道ADC测试验证报告")
        print("="*80)

        # 事件统计
        print("\n1. EVENT事件统计")
        print("-"*80)
        channel_counts = {}
        freq_by_channel = {}
        for event in self.stats['events']:
            ch = event['channel']
            channel_counts[ch] = channel_counts.get(ch, 0) + 1
            if ch not in freq_by_channel:
                freq_by_channel[ch] = []
            freq_by_channel[ch].append(event['freq'])

        print(f"总EVENT数: {len(self.stats['events'])}")
        print("\n各通道触发次数:")
        for ch in sorted(channel_counts.keys()):
            print(f"  {ch}: {channel_counts[ch]} 次")

        # 频率统计
        print("\n2. 频率检测统计")
        print("-"*80)
        for ch in sorted(freq_by_channel.keys()):
            freqs = freq_by_channel[ch]
            if freqs:
                avg = sum(freqs) / len(freqs)
                min_f = min(freqs)
                max_f = max(freqs)
                print(f"  {ch}:")
                print(f"    平均频率: {avg:.1f} Hz")
                print(f"    频率范围: {min_f:.1f} - {max_f:.1f} Hz")
                print(f"    标准差: {self.calculate_std(freqs):.1f} Hz")

        # 去重统计
        print("\n3. 25ms去重统计")
        print("-"*80)
        print(f"去重事件数: {len(self.stats['dedupped'])}")
        if self.stats['dedupped']:
            channel_dedup = {}
            for drop in self.stats['dedupped']:
                ch = drop['channel']
                channel_dedup[ch] = channel_dedup.get(ch, 0) + 1
            print("\n各通道去重次数:")
            for ch in sorted(channel_dedup.keys()):
                print(f"  {ch}: {channel_dedup[ch]} 次")

        # 测试用例统计
        print("\n4. 测试用例执行")
        print("-"*80)
        tc_names = []
        for tc in self.stats['test_cases']:
            if tc['action'] == 'Starting':
                tc_names.append(tc['name'])
        print(f"已执行测试用例: {len(tc_names)} 个")
        if tc_names:
            print("\n测试用例列表:")
            for name in tc_names[:10]:
                print(f"  - {name}")
            if len(tc_names) > 10:
                print(f"  ... 还有 {len(tc_names)-10} 个")

        # 多通道对比
        print("\n5. 多通道一致性")
        print("-"*80)
        if self.stats['compare_data']:
            print("检测到多通道同时触发事件")
            for i, comp in enumerate(self.stats['compare_data'][:3]):
                print(f"\n  对比 #{i+1}:")
                print(f"    {comp['data']}")
        else:
            print("未检测到多通道同时触发")

        # 时间准确性检查
        print("\n6. 时间戳检查")
        print("-"*80)
        if len(self.stats['events']) >= 2:
            # 检查时间递增性
            times = []
            for event in self.stats['events']:
                total_us = event['time_s'] * 1000000 + event['time_ms'] * 1000 + event['time_us']
                times.append(total_us)
            
            increasing = all(times[i] <= times[i+1] for i in range(len(times)-1))
            print(f"时间戳递增: {'✅ 正常' if increasing else '❌ 异常'}")
            
            # 检查时间间隔
            if len(times) >= 3:
                intervals = [times[i+1] - times[i] for i in range(len(times)-1)]
                avg_interval = sum(intervals) / len(intervals)
                print(f"平均事件间隔: {avg_interval/1000:.1f} ms")

        print("\n" + "="*80)

    @staticmethod
    def calculate_std(values):
        if len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return variance ** 0.5

def main():
    print("===== 6通道ADC测试验证 =====")
    print("端口: COM14")
    print("波特率: 2000000")
    print("监控时间: 60秒")
    print("="*80)

    monitor = ADCTestMonitor(port='COM14', baudrate=2000000)
    
    if not monitor.open():
        print("无法打开串口，退出")
        return

    monitor.start_monitor()
    
    try:
        print("\n开始监控... (按 Ctrl+C 停止)")
        for i in range(60):
            time.sleep(1)
            print(f"\r监控中: {i+1}/60 秒", end='')
    except KeyboardInterrupt:
        print("\n\n用户停止监控")
    
    monitor.stop_monitor()
    monitor.close()
    
    monitor.print_summary()

if __name__ == '__main__':
    main()