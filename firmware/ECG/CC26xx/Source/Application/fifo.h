#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>

/*�����־��0-������-1-���*/
#define FLAGS_OVERRUN 0x0001
/*
        buf- ��������ַ
        size- ��С
        free- ��������
        putP- ��һ������д��λ��
        getP- ��һ�����ݶ���λ��
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