#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析测试日志，提取关键信息，记录误差
"""

import re
from collections import defaultdict

def analyze_log(log_file):
    with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
    
    results = {
        'deduplication_events': 0,
        'test_cases': [],
        'channel_data': defaultdict(list),
        'frequency_measurements': defaultdict(list),
        'multi_channel_events': []
    }
    
    current_case = None
    
    for line in lines:
        line = line.strip()
        
        # 检测测试用例开始
        m = re.search(r'\[TEST CASE\] Starting: (.+)', line)
        if m:
            current_case = m.group(1)
            results['test_cases'].append({
                'name': current_case,
                'start': True,
                'events': []
            })
            continue
        
        # 检测测试用例结束
        m = re.search(r'\[TEST CASE\] Finished: (.+)', line)
        if m and results['test_cases']:
            results['test_cases'][-1]['end'] = True
            continue
        
        # 检测去重事件
        if '[DEDUP_DROP]' in line:
            results['deduplication_events'] += 1
            continue
        
        # 检测EVENT事件
        m = re.search(r'>>> EVENT (\S+) T=(\d+)s(\d+)ms(\d+)us freq=([\d.]+)', line)
        if m:
            ch_name = m.group(1)
            ts_s = int(m.group(2))
            ts_ms = int(m.group(3))
            ts_us = int(m.group(4))
            freq = float(m.group(5))
            
            event = {
                'channel': ch_name,
                'time_s': ts_s,
                'time_ms': ts_ms,
                'time_us': ts_us,
                'freq_hz': freq
            }
            
            if results['test_cases'] and not results['test_cases'][-1].get('end', False):
                results['test_cases'][-1]['events'].append(event)
            
            results['channel_data'][ch_name].append(event)
            results['frequency_measurements'][ch_name].append(freq)
            continue
        
        # 检测多通道对比
        m = re.search(r'\[COMPARE\] Multi-ch: (.+)', line)
        if m:
            comp_str = m.group(1)
            results['multi_channel_events'].append(comp_str)
            continue
    
    return results

def print_analysis(results):
    print("="*80)
    print("6通道ADC测试数据分析报告")
    print("="*80)
    
    print("\n1. 测试用例统计")
    print("-"*80)
    for i, case in enumerate(results['test_cases']):
        print(f"   {i+1}. {case['name']}: 触发 {len(case['events'])} 次")
    
    print(f"\n2. 去重机制验证")
    print("-"*80)
    print(f"   25ms内忽略事件数: {results['deduplication_events']}")
    if results['deduplication_events'] > 0:
        print("   ✅ 去重功能正常工作")
    
    print(f"\n3. 各通道触发统计")
    print("-"*80)
    for ch, data in sorted(results['channel_data'].items()):
        freqs = [d['freq_hz'] for d in data]
        if freqs:
            avg_freq = sum(freqs) / len(freqs)
            min_freq = min(freqs)
            max_freq = max(freqs)
            print(f"   {ch}:")
            print(f"      触发次数: {len(data)}")
            print(f"      平均频率: {avg_freq:.1f} Hz")
            print(f"      频率范围: {min_freq:.1f} - {max_freq:.1f} Hz")
    
    print(f"\n4. 多通道一致性分析")
    print("-"*80)
    for i, comp in enumerate(results['multi_channel_events'][:5]):  # 只显示前5个
        print(f"   {i+1}. {comp}")
    if len(results['multi_channel_events']) > 5:
        print(f"   ... 还有 {len(results['multi_channel_events'])-5} 条")
    
    print("\n" + "="*80)

def main():
    log_file = r'F:\ADC_FFT\Myproject\ADC_FFT_TEST\TEST_LOG_20260525_0130.txt'
    results = analyze_log(log_file)
    print_analysis(results)

if __name__ == '__main__':
    main()
