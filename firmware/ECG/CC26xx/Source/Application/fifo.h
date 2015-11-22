#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>

/*溢出标志：0-正常，-1-溢出*/
#define FLAGS_OVERRUN 0x0001
/*
        buf- 缓冲区地址
        size- 大小
        free- 空余容量
        putP- 下一个数据写入位置
        getP- 下一个数据独处位置
*/
struct FIFO8{
    uint16_t *buf;
    int putP,getP,size,free,flags;
};

void fifo8_init(struct FIFO8 *fifo,int size, uint16_t *buf);
uint32_t fifo8_put(struct FIFO8 *fifo,uint16_t data);
uint32_t fifo8_get(struct FIFO8 *fifo);
uint32_t fifo8_status(struct FIFO8 *fifo);
uint32_t fifo8_free(struct FIFO8 *fifo);

#endif