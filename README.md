# GdutRobo-Firmware
广东工业大学机器人创新团队 stm32主控板

## 程序体系结构
application：上层应用任务，如使用FreeRTOS，则包括系统任务

bsp：万用板板载设备适配包

components：通用机器人模块，包括命令行，驱动模块和系统组件

doc：万用板原理图和HAL库文档

MDK-ARM：armcc工具链
