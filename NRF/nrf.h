#ifndef __NRF_H__
#define __NRF_H__
#include "main.h"
#include "spi.h"
#include "nrf_lib.h"


typedef uint8_t nRF24L01_ReceiveAddrHigh_t[4];
typedef uint8_t nRF24L01_ReceiveAddrLow_t;
typedef uint8_t nRF24L01_TransmitAddr_t[5];
typedef struct 
{
    nRF24L01_ReceiveAddrHigh_t high;
    nRF24L01_ReceiveAddrLow_t p1;
    nRF24L01_ReceiveAddrLow_t p2;
    nRF24L01_ReceiveAddrLow_t p3;
    nRF24L01_ReceiveAddrLow_t p4;
    nRF24L01_ReceiveAddrLow_t p5;
} nRF24L01_ReceiveAddress_t;
typedef struct
{
    uint8_t buffer[32];
    uint8_t length;
} nRF24L01_ReceivedData_t;

typedef uint16_t nRF24L01_CE_Pin_t;
typedef GPIO_TypeDef* nRF24L01_CE_Port_t;
typedef uint16_t nRF24L01_NSS_Pin_t;
typedef GPIO_TypeDef* nRF24L01_NSS_Port_t;
typedef uint16_t nRF24L01_IRQ_Pin_t;
typedef GPIO_TypeDef* nRF24L01_IRQ_Port_t;
typedef SPI_HandleTypeDef* nRF24L01_SPI_t;
typedef struct 
{
    uint64_t check_buf[2];
    uint32_t check_index;
} nRF24L01_Check_t;

typedef struct 
{
    nRF24L01_CE_Pin_t ce_pin;
    nRF24L01_CE_Port_t ce_port;
    nRF24L01_NSS_Pin_t nss_pin;
    nRF24L01_NSS_Port_t nss_port;
    nRF24L01_IRQ_Pin_t irq_pin;
    nRF24L01_IRQ_Port_t irq_port;
    nRF24L01_SPI_t hspi;

    nRF24L01_Check_t check;                     //连接性校验
    enum 
    {
        Nrfsignal_Strong,
        Nrfsignal_Weak,
        Nrfsignal_Disconnected
    } signal;
    
    nRF24L01_ReceiveAddress_t receive_address;  //接收地址
    nRF24L01_TransmitAddr_t transmit_address;   //发送地址
    uint8_t rf_channel;                         //射频通道
    nRF24L01_ReceivedData_t receive_data[6];    //接收到的数据
} nRF24L01_t;


typedef enum 
{
    _DisConnected,
    _Connected
} nRF24L01_Connectivity;    //NRF连接状态


extern nRF24L01_t nRF24L01;
extern uint32_t powerShut_cnt;

void nRF24L01_Init(void);                                                   //NRF初始化函数
nRF24L01_Connectivity nRF24L01_CheckConnectivity(void);                     //检查NRF是否连接（读写寄存器）
void nRF24L01_IRQ_EXTI_Callback(uint16_t GPIO_Pin);                         //NRF中断回调函数，用于接收数据
void nRF24L01_Tansmit(uint8_t* tx_buf, uint8_t length);   //发送数据
void nRF24L01_SetTransmitAddress(uint8_t* transmit_addr);   //设置发送地址

#endif // __NRF_H__
