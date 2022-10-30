/**
 * @file average_filter.c
 * @author henry (yuanyu@whut.edu.cn)
 * @brief average filter
 * @version 0.1
 * @date 2022-10-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "average_filter.h"

Average_Filter afp;

/**
 * @brief average filter
 * 
 * @param filter 
 * @param input 
 * @return float 
 */
float average_filter(Average_Filter *filter, float input)
{
    filter->filter_sum -= filter->filter_buf[filter->filter_cnt];
    filter->filter_buf[filter->filter_cnt] = input;
    filter->filter_sum += filter->filter_buf[filter->filter_cnt];
    filter->filter_cnt++;
    if (filter->filter_cnt >= FILTER_NUM)
    {
        filter->filter_cnt = 0;
    }
    return filter->filter_sum / FILTER_NUM;
}

/**
 * @brief average filter init
 * 
 */
void average_filter_init(void)
{
    afp.filter_sum = 0;
    afp.filter_cnt = 0;
    for (uint8_t i = 0; i < FILTER_NUM; i++)
    {
        afp.filter_buf[i] = 0;
    }
}