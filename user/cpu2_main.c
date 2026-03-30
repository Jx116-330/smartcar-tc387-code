/*********************************************************************************************************************
* 文件名称          cpu2_main.c
* 说明              CPU2 核心主函数
* 备注              已按 UTF-8 重新整理注释，程序逻辑保持不变
*********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu2_dsram"





void core2_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 开启全局中断




    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {




    }
}



#pragma section all restore
