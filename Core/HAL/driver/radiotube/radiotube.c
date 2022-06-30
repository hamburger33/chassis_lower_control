#include "radiotube.h"

#include "bsp_def.h"
#include "bsp_delay.h"
#include "bsp_gpio.h"
#include "spi.h"
#include "stdlib.h"
#include "string.h"

//本驱动默认spi数为2且每个spi都只有一组串联的电磁阀驱动板
//因此是一份不完善的驱动，但是对本比赛够用
//完整形态应该是spi数、串联组数、每组驱动板个数均为不定

//改完之后没测过，待测试

cvector *radios;
Spi_io spi_io[2];

void Radiotube_Init() {
    radios = cvector_create(sizeof(radiotube *));
    // spi_io[1].tx_port = HC595_DRCLK;
    // spi_io[1].rx_port = HC165_LD;
}

radiotube *Radiotube_Create(radiotube_config *config) {
    radiotube *obj = (radiotube *)malloc(sizeof(radiotube));
    memset(obj, 0, sizeof(radiotube));
    obj->config = *config;
    // obj->monitor = Monitor_Register(obj->config.lost_callback, 10, obj);
    cvector_pushback(radios, &obj);
    return obj;
}

void Radiotube_Receive(radiotube *obj, uint8_t *radiotube_rx_data) {
    // obj->monitor->reset(obj->monitor);
    uint8_t radio_len = 0;
    for (size_t i = 0; i < radios->cv_len; i++) {
        radiotube *radio = *(radiotube **)cvector_val_at(radios, i);
        if (obj->config.bsp_spi_index == radio->config.bsp_spi_index) radio_len++;  // 遍历，获取该路spi板上的板子数
    }
    BSP_GPIO_Set(spi_io[obj->config.bsp_spi_index].rx_port, 0);  //读取数据到寄存器
    bsp_delay_us(20);
    BSP_GPIO_Set(spi_io[obj->config.bsp_spi_index].rx_port, 1);  //停止读取并允许移位
    bsp_delay_us(20);
    BSP_SPI_Receive(obj->config.bsp_spi_index, radiotube_rx_data, radio_len, 100);  //接收，提供时钟信号，从机接收到时钟信号时开始移位，并发送数据给主机
}
    
void Radiotube_Transmit() {
    static uint8_t data[8] = {0};
    uint8_t radio_len[2] = {0};  //每一串板组的板子个数
    for (size_t spi_id = 0; spi_id < 2; spi_id++) {
        for (size_t i = 0; i < radios->cv_len; i++) {
            radiotube *obj = *(radiotube **)cvector_val_at(radios, i);
            if (obj == NULL)
                return;
            else if (spi_id == obj->config.bsp_spi_index) {
                data[i] = (uint8_t)(obj->data);
                radio_len[spi_id]++;
            }
        }
        if (radio_len[spi_id] != 0) {
            BSP_SPI_Transmit(spi_id, data, radio_len[spi_id], 100);  //发送数据到寄存器
            bsp_delay_us(20);
            BSP_GPIO_Set(spi_io[spi_id].tx_port, 0);  //将移位寄存器的数据载入到存储寄存器中
            bsp_delay_us(20);
            BSP_GPIO_Set(spi_io[spi_id].tx_port, 1);  //更新输出状态（提供一个脉冲）
            bsp_delay_us(20);
            BSP_GPIO_Set(spi_io[spi_id].tx_port, 0);
        }
    }
}