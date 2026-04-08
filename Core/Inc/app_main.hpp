#ifndef APP_MAIN_H
#define APP_MAIN_H

/* 这里的 extern "C" 是给 main.c 看的，让 C 编译器能认识这个函数 */
#ifdef __cplusplus
extern "C" {
#endif

void app_main(void); // 声明你的 C++ 业务入口函数

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */