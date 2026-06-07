
import serial
import serial.tools.list_ports
import time
from datetime import datetime

def find_com_port():
    """查找可用的COM端口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用的COM端口！")
        return None
    
    print("\n可用的COM端口:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    
    # 尝试自动选择USB转串口设备
    for port in ports:
        if any(keyword in port.description.lower() for keyword in ['usb', 'st-link', 'j-link', 'ch340', 'cp210']):
            print(f"\n自动选择: {port.device}")
            return port.device
    
    # 如果没有自动选择，返回第一个
    print(f"\n使用第一个端口: {ports[0].device}")
    return ports[0].device

def capture_serial(port, baudrate=2000000, duration=30, output_file=None):
    """
    抓取串口数据
    
    Args:
        port: COM端口 (如 'COM3')
        baudrate: 波特率 (默认 2000000)
        duration: 抓取时长(秒)，0表示持续抓取直到Ctrl+C
        output_file: 输出文件路径
    """
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"serial_capture_{timestamp}.txt"
    
    print(f"\n开始抓取串口数据...")
    print(f"  端口: {port}")
    print(f"  波特率: {baudrate}")
    print(f"  输出文件: {output_file}")
    if duration > 0:
        print(f"  时长: {duration}秒")
    else:
        print(f"  时长: 持续抓取 (Ctrl+C停止)")
    print("\n" + "="*80)
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        print("串口已打开！\n")
        
        with open(output_file, 'w', encoding='utf-8', buffering=1) as f:
            start_time = time.time()
            
            while True:
                if duration > 0 and (time.time() - start_time) >= duration:
                    print(f"\n\n{duration}秒抓取完成！")
                    break
                
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    try:
                        text = data.decode('utf-8', errors='replace')
                        print(text, end='', flush=True)
                        f.write(text)
                        f.flush()
                    except Exception as e:
                        print(f"\n[解码错误] {e}")
                        hex_data = ' '.join(f'{b:02x}' for b in data)
                        print(f"[原始数据] {hex_data}")
                
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print(f"\n\n用户停止抓取！")
    except Exception as e:
        print(f"\n\n错误: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("\n串口已关闭")
        print(f"数据已保存到: {output_file}")

if __name__ == "__main__":
    print("="*80)
    print("STM32 6通道ADC数据抓取工具")
    print("="*80)
    
    port = find_com_port()
    if port:
        capture_serial(port, baudrate=2000000, duration=0, output_file=None)
