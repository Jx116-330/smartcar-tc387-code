/*********************************************************************************************************************
* 文件名称          isr_config.h
* 说明              中断服务配置头文件
* 备注              已按 UTF-8 重新整理注释，配置数值未改动
*********************************************************************************************************************/

#ifndef _isr_config_h
#define _isr_config_h

#define CCU6_0_CH0_INT_SERVICE  IfxSrc_Tos_cpu0     // 配置 CCU6_0 PIT 通道 0 的中断服务目标，可选 IfxSrc_Tos_cpu0 / IfxSrc_Tos_cpu1 / IfxSrc_Tos_dma，以下同理
#define CCU6_0_CH0_ISR_PRIORITY 50                  // 配置 CCU6_0 PIT 通道 0 的中断优先级，范围 1~255，数值越小优先级越高，建议与常用外设中断错开

#define CCU6_0_CH1_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 51

#define CCU6_1_CH0_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_1_CH0_ISR_PRIORITY 52

#define CCU6_1_CH1_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 53

#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0    // 配置 ERU 通道 0 和通道 4 的中断服务目标，以下同理
#define EXTI_CH0_CH4_INT_PRIO   60                  // 配置 ERU 通道 0 和通道 4 的中断优先级，范围 1~255，数值越小优先级越高

#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0    // 配置 ERU 通道 1 和通道 5 的中断服务目标
#define EXTI_CH1_CH5_INT_PRIO   61                  // 配置 ERU 通道 1 和通道 5 的中断优先级

#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma     // 配置 ERU 通道 2 和通道 6 的中断服务目标
#define EXTI_CH2_CH6_INT_PRIO   5                   // 配置 ERU 通道 2 和通道 6 的中断优先级，DMA 响应范围通常为 0~127

#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu0    // 配置 ERU 通道 3 和通道 7 的中断服务目标
#define EXTI_CH3_CH7_INT_PRIO   62                  // 配置 ERU 通道 3 和通道 7 的中断优先级

#define DMA_INT_SERVICE         IfxSrc_Tos_cpu0     // 配置 DMA 中断服务目标，可选 IfxSrc_Tos_cpu0 / IfxSrc_Tos_cpu1 / IfxSrc_Tos_dma
#define DMA_INT_PRIO            70                  // 配置 DMA 中断优先级，范围 1~255，数值越小优先级越高

#define UART0_INT_SERVICE       IfxSrc_Tos_cpu0     // 配置串口 0 中断服务目标，可选 IfxSrc_Tos_cpu0 / IfxSrc_Tos_cpu1 / IfxSrc_Tos_dma
#define UART0_TX_INT_PRIO       11                  // 配置串口 0 发送中断优先级，范围 1~255，数值越小优先级越高
#define UART0_RX_INT_PRIO       10                  // 配置串口 0 接收中断优先级，范围 1~255，数值越小优先级越高
#define UART0_ER_INT_PRIO       12                  // 配置串口 0 错误中断优先级，范围 1~255，数值越小优先级越高

#define UART1_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART1_TX_INT_PRIO       13
#define UART1_RX_INT_PRIO       14
#define UART1_ER_INT_PRIO       15

#define UART2_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO       16
#define UART2_RX_INT_PRIO       17
#define UART2_ER_INT_PRIO       18

#define UART3_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART3_TX_INT_PRIO       19
#define UART3_RX_INT_PRIO       20
#define UART3_ER_INT_PRIO       21

#define UART4_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART4_TX_INT_PRIO       22
#define UART4_RX_INT_PRIO       23
#define UART4_ER_INT_PRIO       24

#define UART5_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART5_TX_INT_PRIO       25
#define UART5_RX_INT_PRIO       26
#define UART5_ER_INT_PRIO       27

#define UART6_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART6_TX_INT_PRIO       28
#define UART6_RX_INT_PRIO       29
#define UART6_ER_INT_PRIO       30

#define UART8_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART8_TX_INT_PRIO       31
#define UART8_RX_INT_PRIO       32
#define UART8_ER_INT_PRIO       33

#define UART9_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART9_TX_INT_PRIO       34
#define UART9_RX_INT_PRIO       35
#define UART9_ER_INT_PRIO       36

#define UART10_INT_SERVICE      IfxSrc_Tos_cpu0
#define UART10_TX_INT_PRIO      37
#define UART10_RX_INT_PRIO      38
#define UART10_ER_INT_PRIO      39

#define UART11_INT_SERVICE      IfxSrc_Tos_cpu0
#define UART11_TX_INT_PRIO      40
#define UART11_RX_INT_PRIO      41
#define UART11_ER_INT_PRIO      42

#define CCU6_0_CH0_INT_VECTAB_NUM    (int)CCU6_0_CH0_INT_SERVICE      > 0 ? (int)CCU6_0_CH0_INT_SERVICE    - 1 : (int)CCU6_0_CH0_INT_SERVICE
#define CCU6_0_CH1_INT_VECTAB_NUM    (int)CCU6_0_CH1_INT_SERVICE      > 0 ? (int)CCU6_0_CH1_INT_SERVICE    - 1 : (int)CCU6_0_CH1_INT_SERVICE
#define CCU6_1_CH0_INT_VECTAB_NUM    (int)CCU6_1_CH0_INT_SERVICE      > 0 ? (int)CCU6_1_CH0_INT_SERVICE    - 1 : (int)CCU6_1_CH0_INT_SERVICE
#define CCU6_1_CH1_INT_VECTAB_NUM    (int)CCU6_1_CH1_INT_SERVICE      > 0 ? (int)CCU6_1_CH1_INT_SERVICE    - 1 : (int)CCU6_1_CH1_INT_SERVICE

#define EXTI_CH0_CH4_INT_VECTAB_NUM  (int)EXTI_CH0_CH4_INT_SERVICE    > 0 ? (int)EXTI_CH0_CH4_INT_SERVICE  - 1 : (int)EXTI_CH0_CH4_INT_SERVICE
#define EXTI_CH1_CH5_INT_VECTAB_NUM  (int)EXTI_CH1_CH5_INT_SERVICE    > 0 ? (int)EXTI_CH1_CH5_INT_SERVICE  - 1 : (int)EXTI_CH1_CH5_INT_SERVICE
#define EXTI_CH2_CH6_INT_VECTAB_NUM  (int)EXTI_CH2_CH6_INT_SERVICE    > 0 ? (int)EXTI_CH2_CH6_INT_SERVICE  - 1 : (int)EXTI_CH2_CH6_INT_SERVICE
#define EXTI_CH3_CH7_INT_VECTAB_NUM  (int)EXTI_CH3_CH7_INT_SERVICE    > 0 ? (int)EXTI_CH3_CH7_INT_SERVICE  - 1 : (int)EXTI_CH3_CH7_INT_SERVICE

#define DMA_INT_VECTAB_NUM           (int)DMA_INT_SERVICE             > 0 ? (int)DMA_INT_SERVICE           - 1 : (int)DMA_INT_SERVICE

#define UART0_INT_VECTAB_NUM         (int)UART0_INT_SERVICE           > 0 ? (int)UART0_INT_SERVICE         - 1 : (int)UART0_INT_SERVICE
#define UART1_INT_VECTAB_NUM         (int)UART1_INT_SERVICE           > 0 ? (int)UART1_INT_SERVICE         - 1 : (int)UART1_INT_SERVICE
#define UART2_INT_VECTAB_NUM         (int)UART2_INT_SERVICE           > 0 ? (int)UART2_INT_SERVICE         - 1 : (int)UART2_INT_SERVICE
#define UART3_INT_VECTAB_NUM         (int)UART3_INT_SERVICE           > 0 ? (int)UART3_INT_SERVICE         - 1 : (int)UART3_INT_SERVICE
#define UART4_INT_VECTAB_NUM         (int)UART4_INT_SERVICE           > 0 ? (int)UART4_INT_SERVICE         - 1 : (int)UART4_INT_SERVICE
#define UART5_INT_VECTAB_NUM         (int)UART5_INT_SERVICE           > 0 ? (int)UART5_INT_SERVICE         - 1 : (int)UART5_INT_SERVICE
#define UART6_INT_VECTAB_NUM         (int)UART6_INT_SERVICE           > 0 ? (int)UART6_INT_SERVICE         - 1 : (int)UART6_INT_SERVICE
#define UART8_INT_VECTAB_NUM         (int)UART8_INT_SERVICE           > 0 ? (int)UART8_INT_SERVICE         - 1 : (int)UART8_INT_SERVICE
#define UART9_INT_VECTAB_NUM         (int)UART9_INT_SERVICE           > 0 ? (int)UART9_INT_SERVICE         - 1 : (int)UART9_INT_SERVICE
#define UART10_INT_VECTAB_NUM        (int)UART10_INT_SERVICE          > 0 ? (int)UART10_INT_SERVICE        - 1 : (int)UART10_INT_SERVICE
#define UART11_INT_VECTAB_NUM        (int)UART11_INT_SERVICE          > 0 ? (int)UART11_INT_SERVICE        - 1 : (int)UART11_INT_SERVICE

#endif
