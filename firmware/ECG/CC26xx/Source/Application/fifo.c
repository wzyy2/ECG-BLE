#include "fifo.h"

void fifo8_init(struct FIFO8 *fifo,int size, uint16_t *buf)
/*初始化*/
{
    fifo->buf=buf;
    fifo->flags=0;
    fifo->free=size;
    fifo->size=size;
    fifo->putP=0;
    fifo->getP=0;

    return;
}

uint32_t fifo8_put(struct FIFO8 *fifo,uint16_t data)
/*向FIFO 中写入数据 */
{
    if(fifo->free==0){
        fifo->flags |= FLAGS_OVERRUN;
        return -1;
    }
    fifo->buf[fifo->putP] = data;
    fifo->putP++;
    //循环队列缓冲区
    if(fifo->putP == fifo->size){
        fifo->putP = 0;
    }
    fifo->free--;

    return 0;
}

uint32_t fifo8_get(struct FIFO8 *fifo)
/*从FIFO 中取出一个数据 */
{
    uint16_t data;
    if(fifo->free == fifo->size){
        return -1;
    }
    data = fifo->buf[fifo->getP];
    fifo->getP++;
    if(fifo->getP == fifo->size){//用来实现循环
        fifo->getP = 0;
    }
    fifo->free++;

    return data;
}

uint32_t fifo8_status(struct FIFO8 *fifo)
/*缓冲区被使用容量*/
{
    return fifo->size-fifo->free;
}

uint32_t fifo8_free(struct FIFO8 *fifo)
/*缓冲区剩余容量*/
{
    return fifo->free;
}
