#ifndef __THD_H__
#define __THD_H__

#include "main.h"
#include <rtthread.h>

#define THREAD_LIST_NUM     10

typedef struct __THREAD
{
    char thd_name[10];
    rt_thread_t *thd;
    void (*entry)(void *parameter);
    void *parameter;
    rt_uint32_t stack_size;
    rt_uint8_t  priority;
    rt_uint32_t tick;

    struct rt_semaphore *sem;
    char sem_name[10];
    rt_uint32_t sem_value;
    rt_uint8_t  sem_flag;
    rt_uint8_t sem_i_stat;  //sem的初始状态

    rt_mailbox_t mb;
    char mb_name[10];
    rt_size_t mb_size;
    rt_uint8_t mb_flag;
    rt_uint8_t msg;
}thread;

typedef struct __Thread_List
{
    thread *Thread_List[THREAD_LIST_NUM];
    int Thread_List_index;
}thread_list;


#endif
