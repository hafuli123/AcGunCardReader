#include <thd.h>
#include "dev.h"
#include <rtthread.h>
#include <string.h>
#include "bsp_rc522.h"

thread_list global_thd_list;
void Thread_register(thread *thd)
{
    int index = global_thd_list.Thread_List_index;
    global_thd_list.Thread_List[index]=thd;
    index++;
    global_thd_list.Thread_List_index = index;
}
int Thread_find(const char*thd_name)
{
    int i;
    int index = global_thd_list.Thread_List_index;
    for(i=0;i<index;i++){
        if( strcmp(global_thd_list.Thread_List[i]->thd_name , thd_name)==0 ){
            return i;
        }
    }
    return THREAD_LIST_NUM;
}
int Thread_build(const char *pathname, int flags)
{
    int fd;
    thread *thd;
    fd=Thread_find(pathname);
    thd = global_thd_list.Thread_List[fd];
    *(thd->thd) = rt_thread_create(thd->thd_name, thd->entry, thd->parameter,
            thd->stack_size, thd->priority, thd->tick);

    if(thd->sem!=NULL){
        rt_sem_init(thd->sem,thd->sem_name,thd->sem_value,thd->sem_flag);
    }
    if(thd->sem_i_stat==1){
        rt_sem_release(thd->sem);
    }

    if(thd->mb!=NULL){
        thd->mb = rt_mb_create(thd->mb_name,thd->mb_size,thd->mb_flag);
    }
    return fd;
}




rt_thread_t rc522_thd;
struct rt_semaphore rc522_sem;
__IO uint8_t rc522_rxbuf[16];
void rc522_th_entry(void *parameter)
{
    uint8_t r_end,err;
    rt_sem_take(&rc522_sem,RT_WAITING_FOREVER);
    while(1){
        err=rc522_read(0,(uint8_t*)rc522_rxbuf,r_end);
        rt_thread_mdelay(100);

        if(err==MI_OK){
            r_end=1;
        }
        rt_thread_mdelay(10);
    }
}
thread rc522_th={
    "rc522_th",
    &rc522_thd,
    rc522_th_entry,
    NULL,
    2048,
    10,
    5,

    NULL,
    NULL,
    NULL,
    NULL,
    NULL,

    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};


rt_thread_t led_thd;
rt_mailbox_t led_mb;
uint8_t led_mb_val;
struct rt_semaphore led_sem;
void led_th_entry(void *parameter)
{
    while(1){
//        rt_sem_take(&led_sem,RT_WAITING_FOREVER);
//        switch(*(uint8_t*)led_stat+1){
//        case 2:{
//            uint8_t a=LED_ON,*b;
//            b=&a;
//            rt_mb_create(name, size, flag)
//            Device_write(*(uint8_t*)led_stat, b, LED_G);
//            break;
//        }
//        }
        char *str;
        if (rt_mb_recv(led_th.mb, (rt_ubase_t *)&str, RT_WAITING_FOREVER) == RT_EOK){
            HAL_GPIO_WritePin(LED_X_GPIO, LED_G_PIN, GPIO_PIN_RESET);
        }
    }
}

thread led_th={
    "led_th",
    led_thd,
    led_th_entry,
    NULL,
    2048,
    10,
    5,

    NULL,
    NULL,
    NULL,
    NULL,
    NULL,

    led_mb,
    "led_mb",
    1,
    RT_IPC_FLAG_FIFO,
    led_mb_val,
};
