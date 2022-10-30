/**
 * @file average_filter.h
 * @author henry (yuanyu@whut.edu.cn)
 * @brief  average filter
 * @version 0.1
 * @date 2022-10-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef AVERAGE_FILTER_H
#define AVERAGE_FILTER_H

#include "main.h"
#include <stdint.h>
#define FILTER_NUM 10

typedef struct
{
    uint16_t filter_buf[FILTER_NUM];
    uint16_t filter_sum;
    uint8_t filter_cnt;
}Average_Filter;

void average_filter_init(void);
float average_filter(Average_Filter *filter, float input);

extern Average_Filter afp;

#endif