## 移植RT-Thread Nano 到 STM
---
* Clang编译器，将应用代码经过预编译-编译-汇编-链接之后，生成可调试.elf文件。但在main函数中使用操作系统调度级延时函数rt_thread_mdelay()，
clang编译器默认不会把main函数当成单独一个线程，main在此时只是裸机环境，系统上电之后，没等操作系统反应，延时函数就调用，线程挂起，但此时
操作系统还没起来，所以出现了hard_fault_exception错误。
* 这时我们需要修改启动汇编文件，把函数变为先进入entry()线程，在去入口（entry）然后在进入board.c 中初始化rt_hw_board_init()，以及HAL_init
还有系统时钟configuration，接下来启动系统OS调度，系统会自动创建一个main线程，然后在此线程里面去运行main主函数。