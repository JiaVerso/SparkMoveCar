## 移植RT-Thread Nano 到 STM
---
* Clang编译器，将应用代码经过预编译-编译-汇编-链接之后，生成可调试.elf文件。但在main函数中使用操作系统调度级延时函数rt_thread_mdelay()，
clang编译器默认不会把main函数当成单独一个线程，main在此时只是裸机环境，系统上电之后，没等操作系统反应，延时函数就调用，线程挂起，但此时
操作系统还没起来，所以出现了hard_fault_exception错误。
* 这时我们需要修改启动汇编文件，把函数变为先进入entry()线程，在去入口（entry）然后在进入board.c 中初始化rt_hw_board_init()，以及HAL_init
还有系统时钟configuration，接下来启动系统OS调度，系统会自动创建一个main线程，然后在此线程里面去运行main主函数。

/**
 ******************************************************************************
 **/
 ### UART
* DMA半满全满+环形缓冲区(FIFO)+idle
!!! malloc 内存池 内存管理块 ---（预编译指令（ASSERT断言） #ifdef USE_DYNAMIC_MEMORY等等）--- 指针函数fifo_s_t* app_create() 和函数指针fifo_s_t *app_create()
!!! 互斥锁 MUTEX_DECLARE(mutex)
!!! memcpy（无法处理内存重叠，本质是从前到后的按字节搬运（不确定底层原理）） memmove（多了一层地址判断） 区别
issue: 串口接受乱码： 时钟配置错误，假如实际晶振8，cubemx晶振25，速度就会慢3倍，115200通信率就会降低3倍，所以接收乱码
2. 内部时钟pll倍频不稳定（HSI 的频率会发生明显的漂移（误差通常在 1% 到 3% 左右）），并且时钟频率太低，串口波特率无法拉高
3. 数据频率太高，fifo被填满,free_num 为0， return 0.丢包，然后等发送函数处理完，释放空余空间
4. 指针回绕问题，在回调函数中解决，判断写指针和buf_pos的位置，去进行分段puts