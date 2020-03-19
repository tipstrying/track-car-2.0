# track-car-MC

######  520 专案 MCU 主程序

* 基础信息
  + 软件
    - 操作系统：FreeRTOS  V10.0.1
    - 文件系统：fatfs R0.12c
    - 网络库：wizchip 1.0.1
    - 驱动：STM32 HAL库
  + 硬件
    - 主控：stm32f429
    - 网络适配器：w5500
    - 外置存储：无

* 网络信息
  + 默认IP：192.168.1.216（可使用命令行修改运行时IP，运行时IP存储在RTC DR0）
  + 控制端口：8802
  + 日志输出端口：5001
  + 命令行端口：5002

