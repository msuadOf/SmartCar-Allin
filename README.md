# STC赛道智能车

## 引脚接线与外设使用

| 功能                    | 外设         | 引脚                                                         | 备注                           |
| ----------------------- | ------------ | ------------------------------------------------------------ | ------------------------------ |
| UART调试串口            | UART2        | RX:P4.6<br />TX:P4.7                                         | 串口需要设置为准双向口         |
| 电机1                   | PWMA_CH1     | PWMA_CH1P：P60<br />PWMA_CH1N：P61                           | 设置为推挽                     |
| 电机2                   | PWMA_CH2     | PWMA_CH2P：P62<br />PWMA_CH2N：P63                           | 设置为推挽                     |
| 编码器1                 | TIM0_P34     | TIM0:P34(A)<br />GPIO:P33(B)                                 |                                |
| 编码器2                 | TIM1_P35     | TIM1:P35(A)<br />GPIO:P36(B)                                 |                                |
| TIM+DMA+ADC定时采集中断 | TIM4+DMA+ADC |                                                              | 所有实时算法都在这个中断中实现 |
| MPU6500通讯底层         | SPI          | NCS：P22：<br />MOSI：P23（SDA）<br />MISO：P24（ADO）<br />SCLK：P25（SCK） |                                |
|                         |              |                                                              |                                |
|                         |              |                                                              |                                |
|                         |              |                                                              |                                |
|                         |              |                                                              |                                |
|                         |              |                                                              |                                |
|                         |              |                                                              |                                |




