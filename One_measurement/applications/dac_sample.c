/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-28     袁雨       the first version
 */
/*
 * 程序清单： DAC和ADC设备测试例程
 * 例程导出了 vol_read_sample 命令到控制终端
 * 命令调用格式： vol_read_sample
 * 程序功能：通过 DAC 设备将数字值3000转换为模拟量，并输出电压值2.42V。
 *           示例代码参考电压为3.3V,转换位数为12位。
*/

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "Kalman/Kalman.h"
#include "filter/average_filter/average_filter.h"

#define DAC_DEV_NAME        "dac1"  /* DAC 设备名称 */
#define DAC_DEV_CHANNEL     1       /* DAC 通道 */
#define REFER_VOLTAGE       330         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */

#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */
#define ADC_DEV_CHANNEL     13           /* ADC 通道 */


static int vol_read_sample(int argc, char *argv[])
{
    rt_dac_device_t dac_dev;
    rt_adc_device_t adc_dev;
    rt_uint32_t value, vol, kalman_vol, average_vol;
    rt_err_t ret = RT_EOK;
    rt_uint8_t cnt = 0;
    char buf[20] = {0};
    rt_size_t send_len = 0;


    Kalman_Init();

    /* 查找设备 */
    dac_dev = (rt_dac_device_t)rt_device_find(DAC_DEV_NAME);
    if (dac_dev == RT_NULL)
    {
        rt_kprintf("dac sample run failed! can't find %s device!\n", DAC_DEV_NAME);
        return RT_ERROR;
    }
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 打开设备 */
    ret = rt_dac_enable(dac_dev, DAC_DEV_CHANNEL);
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);

    /* 设置输出值 */
    value = 3000;
    rt_dac_write(dac_dev, DAC_DEV_CHANNEL, value);
    rt_kprintf("the value is :%d \n", value);

    /* 转换为对应电压值 */
    vol = value * REFER_VOLTAGE / CONVERT_BITS;
    rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);

    while(1)
    {
        /* 读取ADC值 */
        value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
        // rt_kprintf("the adc value is :%d \n", value);
        /* 转换为对应电压值 */
        vol = value * REFER_VOLTAGE / CONVERT_BITS;
        kalman_vol = KalmanFilter(&kfp, vol);
        average_vol = (rt_uint32_t)average_filter(&afp, (float)vol);
        // rt_kprintf("the adc voltage is :%d.%02d \n", vol / 100, vol % 100);
        rt_kprintf("%d.%02d, %d.%02d, %d.%02d\n", vol / 100, vol % 100, kalman_vol / 100, kalman_vol % 100, average_vol / 100, average_vol % 100);
        rt_thread_mdelay(200);
    }

    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(vol_read_sample, voltage convert and read sample);
// 导出到init
// INIT_APP_EXPORT(vol_read_sample);