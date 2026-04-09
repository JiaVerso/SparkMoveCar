#include "app_main.hpp"
#include "main.h"      // 包含 HAL 库头文件
#include <rtthread.h>  // 如果用到 RT-Thread

// 你可以在这里自由使用 C++ 的类、对象、STL 模板等
class MyRobot {
public:
    void moveForward() {
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }
};

MyRobot robot;

// 定义真正的业务入口
void app_main(void) 
{
    // 这里就是你 C++ 世界的起点！
    while(1) {
        robot.moveForward();
        rt_thread_mdelay(500);
    }
}

