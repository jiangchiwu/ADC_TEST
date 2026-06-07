
@echo off
REM STM32 6通道ADC串口数据抓取工具
REM 使用2Mbps波特率

echo ========================================================
echo STM32 6通道ADC实时采集 - 串口数据抓取
echo ========================================================
echo.

python -c "import serial.tools.list_ports; print('可用COM端口:'); [print(f'  {p.device} - {p.description}') for p in serial.tools.list_ports.comports()]" 2>nul

if %errorlevel% neq 0 (
    echo.
    echo [错误] 未找到Python或pyserial库！
    echo 请先安装: pip install pyserial
    echo.
    pause
    exit /b 1
)

echo.
echo 请确保:
echo   1. STM32开发板已连接并运行程序
echo   2. 串口线已连接 (UART7, PE7-TX/PE8-RX)
echo   3. USB转串口驱动已安装
echo.
echo 按任意键开始抓取串口数据 (2Mbps)...
pause >nul

python serial_capture.py

pause
