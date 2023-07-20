/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-06-25     RT-Thread    first version
 */

#include <rtthread.h>
#include <string.h>
#include "main.h"
#include "gpio.h"
#include "dev.h"
#include "bsp_rc522.h"
#include "usart.h"
#include "bsp_spiflash.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

struct __Config{
    int ip; //例如ip=0x1122，则信道0x11，id0x22
    uint8_t ch;
    uint8_t id;

    //暂存
    int ip_temp;
    uint8_t ch_temp;
    uint8_t id_temp;
}config;


__IO int fd_rc522, fd_led, fd_gc24, fd_spiflash;
__IO uint8_t gc24_stat;
__IO uint8_t rc522_rxbuf[16] ;
__IO uint8_t flash_rxbuf[4] , flash_txbuf[4];
__IO uint8_t atmod0[]="AT+MODE=0\r\n" , atmod1[]="AT+MODE=1\r\n" , atrn[]="\r\n";
__IO uint8_t work_stat; //工作状态
__IO uint8_t reset_stat;

extern uint8_t rc522_KeyB[6] , rc522_ip[16];

uint8_t led_g=LED_G, led_r = LED_R ,led_y = LED_Y ,led_ysk = LED_Y_SK,led_rsk = LED_R_SK;

Thread rc522_th,led_th,gc_send_th,gc_recv_th,gc_ioctl_th,gc_chk , key_th , ipad_th;



int main(void)
{
    HAL_Init();
    MX_GPIO_Init();

    rt_sec_delay(1);    //延时，这样可实现长按开机
    SYSTEM_POWER_ON();

    Device_register(&rc522);    //注册并开启设备
    Device_register(&led);
    Device_register(&gc24);
    Device_register(&spiflash);

    fd_rc522=Device_open("rc522",0);
    fd_led=Device_open("led",0);
    fd_gc24=Device_open("gc24",0);
    fd_spiflash=Device_open("spiflash",0);

    //创建子线程
    led_th.th = rt_thread_create("led_th", led_th_entry, NULL, 512, 10, 5);
    rt_thread_startup(led_th.th);
    rt_sem_init(&led_th.sem, "led_sem", 0, RT_IPC_FLAG_FIFO);
    led_th.mb = rt_mb_create("led_mb", 1, RT_IPC_FLAG_FIFO);

    key_th.th = rt_thread_create("key_th", key_th_entry, NULL, 1024, 10, 5);
    rt_thread_startup(key_th.th);
    rt_sem_init(&key_th.sem, "key_th_sem", 1, RT_IPC_FLAG_FIFO);

    work_stat = WORK_STAT_INIT;
    rc522_th.th = rt_thread_create("rc522_th", rc522_th_entry, NULL, 2048, 10, 5);
    rt_thread_startup(rc522_th.th);
    rt_sem_init(&rc522_th.sem, "rc522_sem", 0, RT_IPC_FLAG_FIFO);

    gc_chk.th = rt_thread_create("gc_chk_th", gc_chk_th_entry, NULL, 512, 10, 5);
    rt_thread_startup(gc_chk.th);
    gc_chk.tm = rt_timer_create("gc_chk_tm", gc_chk_tm_callback, NULL, 5000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);  //等待靶面回复等待5s
    rt_sem_init(&gc_chk.sem, "gc_chk_sem", 0, RT_IPC_FLAG_FIFO);

    gc_send_th.th = rt_thread_create("gc_send_th", gc_send_th_entry, NULL, 1024, 10, 5);
    rt_thread_startup(gc_send_th.th);
    rt_sem_init(&gc_send_th.sem, "gc_send_sem", 0, RT_IPC_FLAG_FIFO);

    gc_recv_th.th = rt_thread_create("gc_recv_th", gc_recv_th_entry, NULL, 1024, 10, 5);
    rt_thread_startup(gc_recv_th.th);
    rt_sem_init(&gc_recv_th.sem, "gc_recv_sem", 0, RT_IPC_FLAG_FIFO);

    gc_ioctl_th.th = rt_thread_create("gc_ioctl_th", gc_ioctl_th_entry, NULL, 2048, 10, 5);
    rt_thread_startup(gc_ioctl_th.th);
    rt_sem_init(&gc_ioctl_th.sem, "gc_ioctl_sem", 0, RT_IPC_FLAG_FIFO);
    gc_ioctl_th.mb = rt_mb_create("gc_ioctl_mb", 1, RT_IPC_FLAG_FIFO);

    //开局需要获取当前无线模块的CH ID，得知IP
    gc24_stat=0xff;
    Device_write(fd_gc24, (uint8_t*)atmod0,strlen((char*)atmod0));

    return RT_EOK;
}

void key_th_entry(void *parameter)
{
    uint8_t stat[4];
    while(1){
        rt_thread_mdelay(100);
        if(KEY_READ(KEY_ON_PIN)==KEY_ON_DOWN_LEVEL){
            rt_sec_delay(3);
            if(KEY_READ(KEY_ON_PIN)==KEY_ON_DOWN_LEVEL){
                if(work_stat == WORK_STAT_IPAD_FREE){
                    work_stat = WORK_STAT_WAIT_RESET;
                    rt_mb_send(led_th.mb, *(uint8_t*)&led_r);
                    rt_sem_release(&led_th.sem);
                }
                else if(work_stat == WORK_STAT_WAIT_RESET){
                    work_stat = WORK_STAT_IPAD_FREE;
                    rt_mb_send(led_th.mb, *(uint8_t*)&led_y);
                    rt_sem_release(&led_th.sem);
                }
                else{
                    SYSTEM_POWER_OFF();
                }

            }
            continue;
        }
        if(KEY_READ(KEY_2_PIN)==KEY_ON_DOWN_LEVEL){
            rt_thread_mdelay(5);
            if(KEY_READ(KEY_2_PIN)==KEY_ON_DOWN_LEVEL){
                if(work_stat == WORK_STAT_FREE){
                    work_stat = WORK_STAT_IPAD_FREE;
                    reset_stat=0;
                    stat[0]=0xDD; stat[1]=0x01; stat[2]=0x02;stat[3]=stat[0]+stat[1]+stat[2];
                    Device_write(fd_gc24, (uint8_t*)stat, 4);
                    rt_mb_send(led_th.mb, *(uint8_t*)&led_y);
                    rt_sem_release(&led_th.sem);
                    while(KEY_READ(KEY_2_PIN)==KEY_ON_DOWN_LEVEL){rt_thread_mdelay(2);}
                }
                else if(work_stat == WORK_STAT_IPAD_FREE){
                    rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                    rt_sem_release(&led_th.sem);
                    work_stat = WORK_STAT_FREE;
                    while(KEY_READ(KEY_2_PIN)==KEY_ON_DOWN_LEVEL){rt_thread_mdelay(2);}
                }
                else if(work_stat == WORK_STAT_WAIT_RESET){
                    reset_stat++;
                    if(reset_stat==8){
                        reset_stat=0;
                        work_stat = WORK_STAT_RESET;
                        rt_mb_send(led_th.mb, *(uint8_t*)&led_rsk);
                        rt_sem_release(&led_th.sem);
                        gc24_stat=0xef;
                        config.ip=0;
                        rt_thread_mdelay(100);
                        Device_write(fd_gc24, (uint8_t*)atmod0, strlen((char *)atmod0));
                        continue;
                    }
                    while(KEY_READ(KEY_2_PIN)==KEY_ON_DOWN_LEVEL){rt_thread_mdelay(2);}
                }
            }
            continue;
        }

        if(work_stat == WORK_STAT_FREE){
            if(BAT_CHRG_Y){
                work_stat = WORK_STAT_CHARG;
                rt_mb_send(led_th.mb, *(uint8_t*)&led_r);
                rt_sem_release(&led_th.sem);
                continue;
            }
        }
        else if(work_stat == WORK_STAT_CHARG){
            if(BAT_CHRG_N){
                work_stat=WORK_STAT_FREE;
                rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                rt_sem_release(&led_th.sem);
                continue;
            }
            else if(BAT_DONE_Y){
                if(Device_read(fd_led, NULL, LED_R)==LED_ON){
                    rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                    rt_sem_release(&led_th.sem);
                }
                continue;
            }
        }


    }
}

void led_th_entry(void *parameter)
{
    uint8_t stat;uint8_t on,off;
    on=LED_ON; off=LED_OFF;
    while(1){
        rt_sem_take(&led_th.sem,RT_WAITING_FOREVER);
        if (rt_mb_recv(led_th.mb, (rt_ubase_t *)&stat, RT_WAITING_FOREVER) == RT_EOK){
            Device_write(fd_led, &off, LED_G);
            Device_write(fd_led, &off, LED_R);
            Device_write(fd_led, &off, LED_Y);
            if(stat<LED_R_SK){
                Device_write(fd_led, &on, stat);
            }
            else{
                for(int i=0;i<2;i++){
                    Device_write(fd_led, &on, stat-LED_R_SK);
                    rt_thread_mdelay(50);
                    Device_write(fd_led, &off, stat-LED_R_SK);
                    rt_thread_mdelay(50);
                    Device_write(fd_led, &on, stat-LED_R_SK);
                }
            }
        }
    }
}

void rc522_th_entry(void *parameter)
{
    uint8_t rcReturn;
    while(1){
        rt_sem_take(&rc522_th.sem,RT_WAITING_FOREVER);
        while(1){
            rt_thread_mdelay(500);
            if(work_stat != WORK_STAT_FREE){
                continue;
            }
            memset((uint8_t*)rc522_rxbuf,0,16);
            rcReturn = Device_read(fd_rc522,(uint8_t*)rc522_rxbuf,0);
            if(rcReturn == MI_NOTAGERR){
                rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                rt_sem_release(&led_th.sem);
                continue;
            }
            else if(rcReturn == MI_ERR){
                rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                rt_sem_release(&led_th.sem);
                continue;
            }
            else if(rcReturn == MI_OK){
                rt_mb_send(led_th.mb, *(uint8_t*)&led_y);
                rt_sem_release(&led_th.sem);
                rt_thread_mdelay(50);
                if(*rc522_rxbuf==0xAA){
                    //开始修改靶面地址
                    work_stat = WORK_STAT_MIS;
                    rt_sem_release(&gc_send_th.sem);
                    break;
                }
                continue;
            }
        }

    }
}

void gc_send_th_entry(void *parameter)
{
    uint16_t *p ;
    while(1){
        rt_sem_take(&gc_send_th.sem,RT_WAITING_FOREVER);
        p=(uint16_t*)(rc522_rxbuf+1);
        if(config.ip == (int)(*p)){
            //枪卡地址和读卡器一样，就不需要修改了
            work_stat = WORK_STAT_FREE;
            rt_thread_mdelay(50);
            rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
            rt_sem_release(&led_th.sem);
            memset((uint8_t*)rc522_rxbuf,0,16);
            rt_sem_release(&rc522_th.sem);
            continue;
        }
        else{
            //给靶面发送枪卡的地址信息
            Device_write(fd_gc24, (uint8_t*)rc522_rxbuf, 4);
            rt_thread_mdelay(100);
            //改配置,使得读卡器和枪卡地址一样
            config.ip_temp = (int)(*p);
            config.id_temp = (uint8_t)(*p&0x000000ff);
            config.ch_temp = (uint8_t)( (*p&0x0000ff00)>>8 );
            Device_write(fd_gc24, (uint8_t*)atmod0, strlen((char *)atmod0));
            gc24_stat=0x0f;
            continue;
        }

    }
}

void gc_ioctl_th_entry(void*parameter)
{
    uint8_t *stat , tx[4];
    uint8_t atp[20],atpid_[]="AT+PID=",atch_[]="AT+RFCH=",at_rn[]="\r\n" , at_q[]="?\r\n";

    uint8_t *pid,*q,*d,atpidsize;
    while(1){
        rt_sem_take(&gc_ioctl_th.sem,RT_WAITING_FOREVER);
        atpidsize=0;
        if (rt_mb_recv(gc_ioctl_th.mb, (rt_ubase_t *)&stat, RT_WAITING_FOREVER) == RT_EOK){
            if((*stat==0x01)||(*stat==0x11)||(*stat==0x21)||(*stat==0xe1)){

                if(*stat==0x11){
                    //恢复
                    config.id_temp = (uint8_t)(config.ip&0x000000ff);
                    config.ch_temp = (uint8_t)( (config.ip&0x0000ff00)>>8 );
                }
                else if(*stat==0xe1){
                    config.id_temp = 0;
                    config.ch_temp = 0;

                }

                //修改PID
                pid=atpid_;q=atp;d=at_rn;atpidsize=0;
                for(int i=0;i<strlen((char*)atpid_);i++){
                    *q=*pid;
                    q++;pid++;atpidsize++;
                }
                if(config.id_temp<10){
                    *q=(0x30|config.id_temp);atpidsize++;q++;
                }
                else if((config.id_temp>=10)&&(config.id_temp<100)){
                    *q=(0x30|(config.id_temp/10%10) );atpidsize++;q++;
                    *q=(0x30|(config.id_temp%10) );atpidsize++;q++;
                }
                else if((config.id_temp>=100)&&(config.id_temp<=0xff)){
                    *q=(0x30|(config.id_temp/100%10) );atpidsize++;q++;
                    *q=(0x30|(config.id_temp/10%10) );atpidsize++;q++;
                    *q=(0x30|(config.id_temp%10) );atpidsize++;q++;
                }
                for(int i=0;i<strlen((char *)at_rn);i++){
                    *q=*d;
                    q++;d++;atpidsize++;
                }
                Device_write(fd_gc24, atp, atpidsize);
                continue;
            }
            else if(*stat==0x20){
                rt_thread_mdelay(100);
                Device_write(fd_gc24, (uint8_t*)atmod0, strlen((char *)atmod0));
                continue;
            }
            else if((*stat==0x02)||(*stat==0x12)||(*stat==0x22)||(*stat==0xe2)){
                //修改RFCH
                pid=atch_;q=atp;d=at_rn;atpidsize=0;
                for(int i=0;i<strlen((char*)atch_);i++){
                    *q=*pid;
                    q++;pid++;atpidsize++;
                }
                if(config.ch_temp<10){
                    *q=(0x30|config.ch_temp);atpidsize++;q++;
                }
                else if((config.ch_temp>=10)&&(config.ch_temp<100)){
                    *q=(0x30|(config.ch_temp/10%10) );atpidsize++;q++;
                    *q=(0x30|(config.ch_temp%10) );atpidsize++;q++;
                }
                for(int i=0;i<strlen((char *)at_rn);i++){
                    *q=*d;
                    q++;d++;atpidsize++;
                }
                Device_write(fd_gc24, atp, atpidsize);
                continue;
            }
            else if((*stat==0x03)||(*stat==0x13)||(*stat==0x23)||(*stat==0xe3)){
                //IP配置修改完成，发送AT+MODE=1
                Device_write(fd_gc24, (uint8_t*)atmod1,strlen((char*)atmod1));
//                Device_write(fd_gc24, (uint8_t*)atrn,strlen((char*)atrn));
                continue;
            }
            //结束类
            else if((*stat==0x04)||(*stat==0x15)||(*stat==0x24)||(*stat==0x05)||(*stat==0x06)||(*stat==0xe4)){
                if(*stat==0x04){
                    //等待靶的回复
                    rt_timer_start(gc_chk.tm);
                    continue;
                }
                else if(*stat==0x24){
                    //回复IPAD靶，并等待IPAD靶回复
                    rt_thread_mdelay(500);  //先等待一会确保靶面的无线模块配置改好了，再回复
                    uint8_t *t;t=(uint8_t*)&config.ip_temp;
                    tx[0]=0xaa;tx[1]=*t;tx[2]=*(t+1);tx[3]=tx[0]+tx[1]+tx[2];
                    for(int i=0;i<5;i++){
                        Device_write(fd_gc24, (uint8_t*)tx, 4);
                        rt_thread_mdelay(2);
                    }
                    rt_timer_start(gc_chk.tm);
                    continue;
                }
                else if(*stat==0x15){
                    //结束，回复靶几条
                    gc24_stat=0x00;
                    config.ip_temp=0;
                    config.id_temp = 0;
                    config.ch_temp = 0;


                    for(int i=0;i<5;i++){
                        Device_write(fd_gc24, (uint8_t*)rc522_rxbuf, 4);
                        rt_thread_mdelay(2);
                    }

                    rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                    rt_sem_release(&led_th.sem);

                    rt_sec_delay(5);
                    memset((uint8_t*)rc522_rxbuf,0,16);
                    work_stat = WORK_STAT_FREE;
                    rt_sem_release(&rc522_th.sem);
                    continue;
                }
                else if((*stat==0x05)||(*stat==0x06)){
                    //结束，不需回复
                    gc24_stat=0x00;
                    config.ip_temp=0;
                    config.id_temp = 0;
                    config.ch_temp = 0;

                    if(work_stat == WORK_STAT_IPAD_WORK){
                        rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                        rt_sem_release(&led_th.sem);
                        work_stat = WORK_STAT_FREE;
                        continue;
                    }
                    //读卡修改失败
                    rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                    rt_sem_release(&led_th.sem);
                    rt_sec_delay(5);
                    memset((uint8_t*)rc522_rxbuf,0,16);
                    work_stat = WORK_STAT_FREE;
                    rt_sem_release(&rc522_th.sem);
                    continue;
                }
                else if(*stat==0xe4){
                    //RESET结束
                    gc24_stat=0x00;
                    rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                    rt_sem_release(&led_th.sem);
                    work_stat = WORK_STAT_FREE;
                    continue;
                }

            }
            //初始化类
            else if(*stat==0xf1){
                pid=atpid_;q=atp;d=at_q;atpidsize=0;
                for(int i=0;i<strlen((char*)atpid_);i++){
                    *q=*pid;
                    q++;pid++;atpidsize++;
                }
                for(int i=0;i<strlen((char*)at_q);i++){
                    *q=*d;
                    q++;d++;atpidsize++;
                }
                Device_write(fd_gc24, atp, atpidsize);
                continue;
            }
            else if(*stat==0xf2){
                pid=atch_;q=atp;d=at_q;atpidsize=0;
                for(int i=0;i<strlen((char*)atch_);i++){
                    *q=*pid;
                    q++;pid++;atpidsize++;
                }
                for(int i=0;i<strlen((char*)at_q);i++){
                    *q=*d;
                    q++;d++;atpidsize++;
                }
                Device_write(fd_gc24, atp, atpidsize);
                continue;
            }
            else if(*stat==0xf3){
                //成功获取IP
                config.ip=(((uint16_t)config.ch)<<8)|((uint16_t)config.id);
                Device_write(fd_gc24, (uint8_t*)atmod1,strlen((char*)atmod1));
            }
            else if(*stat == 0xf4){
                //初始化IP获取结束
                gc24_stat=0;
                rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                rt_sem_release(&led_th.sem);
                work_stat = WORK_STAT_FREE;
                rt_sem_release(&rc522_th.sem);
                                                //        config.ip=0x0A5B;   //CH 10 ID 91
                Device_write(fd_gc24, (uint8_t*)"ok",2);
                continue;
            }
        }
    }
}

void gc_recv_th_entry(void *parameter)
{
    uint8_t rxbuf[RX_MAX_COUNT];
    while(1){
//        rt_sem_take(&gc_recv_th.sem,RT_WAITING_FOREVER);
        rt_thread_mdelay(1);
        Device_read(fd_gc24, rxbuf, 0);
        if((*rxbuf=='O')&&(rxbuf[1]=='K')){
            switch(gc24_stat){
            case 0xff:{gc24_stat=0xf1;break;}
            case 0xf3:{gc24_stat=0xf4;break;}//

            case 0x0f:{gc24_stat=0x01;break;}
            case 0x10:{gc24_stat=0x11;break;}
            case 0x20:{gc24_stat=0x21;break;}
            case 0xef:{gc24_stat=0xe1;break;}

            case 0x01:{gc24_stat=0x02;break;}
            case 0x11:{gc24_stat=0x12;break;}
            case 0x21:{gc24_stat=0x22;break;}
            case 0xe1:{gc24_stat=0xe2;break;}

            case 0x02:{gc24_stat=0x03;break;}
            case 0x12:{gc24_stat=0x13;break;}
            case 0x22:{gc24_stat=0x23;break;}
            case 0xe2:{gc24_stat=0xe3;break;}

            case 0x03:{gc24_stat=0x04;break;}
            case 0x13:{gc24_stat=0x05;break;}
            case 0x23:{gc24_stat=0x24;break;}
            case 0xe3:{gc24_stat=0xe4;break;}
            }
            rt_mb_send(gc_ioctl_th.mb, (rt_uint32_t)&gc24_stat);
            rt_sem_release(&gc_ioctl_th.sem);
            memset(rxbuf,0,RX_MAX_COUNT) ;
            continue;
        }
        else if((rxbuf[0]=='A')&&(rxbuf[1]=='T')&&(work_stat == WORK_STAT_INIT)){
            switch(gc24_stat){
            case 0xf1:{
                gc24_stat=0xf2; //PID
                if(*(rxbuf+8)=='\r'){   //ID只有个位
                    config.id = *(rxbuf+7)&0x0f;
                }
                else if(*(rxbuf+9)=='\r'){  //ID有十位
                    config.id = ((*(rxbuf+7)&0x0f)*10)+(*(rxbuf+8)&0x0f);
                }
                else if(*(rxbuf+10)=='\r'){  //ID有百位
                    config.id = ((*(rxbuf+7)&0x0f)*100) + ((*(rxbuf+8)&0x0f)*10) + (*(rxbuf+9)&0x0f);
                }
                break;
            }
            case 0xf2:{
                gc24_stat=0xf3; //CH
                if(*(rxbuf+7)=='\r'){   //CH只有个位
                    config.ch = 0x0f&(*(rxbuf+6));
                }
                else if(*(rxbuf+8)=='\r'){   //CH只有十位
                    config.ch = (10*(0x0f&(*(rxbuf+6)))) + (0x0f&(*(rxbuf+7)));
                }
                break;
            }
            }
            rt_mb_send(gc_ioctl_th.mb, (rt_uint32_t)&gc24_stat);
            rt_sem_release(&gc_ioctl_th.sem);
            memset(rxbuf,0,RX_MAX_COUNT) ;
            continue;
        }
        else if((*rxbuf==0xAA)&&(gc24_stat==0x04)){
            //成功收到靶面发来的回复,回发5次
            rt_timer_stop(gc_chk.tm);
            //存起更改后的ip
            config.ip = config.ip_temp;
            config.id= config.id_temp;
            config.ch=config.ch_temp;

            gc24_stat=0x15;
            rt_mb_send(gc_ioctl_th.mb, (rt_uint32_t)&gc24_stat);
            rt_sem_release(&gc_ioctl_th.sem);
            memset(rxbuf,0,RX_MAX_COUNT) ;
            continue;
        }
        else if((work_stat == WORK_STAT_IPAD_FREE)&&(*rxbuf==0xAA)){    //收到ipad的靶面发来的修改地址信息
            if(config.ip ==  (int)( *(uint16_t*)(rxbuf+1) )){
                work_stat = WORK_STAT_FREE;
                rt_mb_send(led_th.mb, *(uint8_t*)&led_g);
                rt_sem_release(&led_th.sem);
                Device_write(fd_gc24, (uint8_t*)rxbuf, 4);
                memset(rxbuf,0,RX_MAX_COUNT) ;
                continue;
            }
            rt_mb_send(led_th.mb, *(uint8_t*)&led_ysk);
            rt_sem_release(&led_th.sem);
            config.ip_temp = (int)( *(uint16_t*)(rxbuf+1) );
            config.id_temp = (uint8_t)( (*(uint16_t*)(rxbuf+1)) &0x000000ff);
            config.ch_temp = (uint8_t)( ( (*(uint16_t*)(rxbuf+1)) &0x0000ff00)>>8 );
            work_stat = WORK_STAT_IPAD_WORK;
            gc24_stat=0x20;
            rt_mb_send(gc_ioctl_th.mb, (rt_uint32_t)&gc24_stat);
            rt_sem_release(&gc_ioctl_th.sem);
            memset(rxbuf,0,RX_MAX_COUNT) ;
            continue;
        }
        else if((gc24_stat==0x24)&&(*rxbuf==0xAA)){
            //成功收到IPAD的靶发来的回复
            rt_timer_stop(gc_chk.tm);
            config.ip = config.ip_temp;
            config.ch = config.ch_temp;
            config.id=config.id_temp;
            gc24_stat=0x06;
            rt_mb_send(gc_ioctl_th.mb, (rt_uint32_t)&gc24_stat);
            rt_sem_release(&gc_ioctl_th.sem);

            memset(rxbuf,0,RX_MAX_COUNT) ;
            continue;
        }
        memset(rxbuf,0,RX_MAX_COUNT) ;
    }

}

void gc_chk_tm_callback(void*parameter)
{
    rt_sem_release(&gc_chk.sem);
}
void gc_chk_th_entry(void *parameter)
{
    //接收回复失败，没有收到靶面回复的信息，读卡器恢复原来的配置
    while(1){
        rt_sem_take(&gc_chk.sem, RT_WAITING_FOREVER);
        gc24_stat=0x10;
        Device_write(fd_gc24, (uint8_t*)atmod0, strlen((char *)atmod0));
        rt_mb_send(led_th.mb, *(uint8_t*)&led_rsk);
        rt_sem_release(&led_th.sem);
    }
}



void rt_sec_delay(int sec)
{
    for(int i=0;i<sec;i++){
        rt_thread_mdelay(1000);
    }
}

