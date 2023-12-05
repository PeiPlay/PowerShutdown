#ifndef __NRF_LIB_H__
#define __NRF_LIB_H__

//nRF24L01寄存器操作命令
#define NRF_CMD_READ_REG                0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_CMD_WRITE_REG               0x20  //写配置寄存器,低5位为寄存器地址
#define NRF_CMD_READ_RX_PAYLOAD         0x61  //读RX有效数据,1~32字节
#define NRF_CMD_WRITE_TX_PAYLOAD        0xA0  //写TX有效数据,1~32字节
#define NRF_CMD_FLUSH_TX                0xE1  //清除TX FIFO寄存器.发射模式下用
#define NRF_CMD_FLUSH_RX                0xE2  //清除RX FIFO寄存器.接收模式下用，,在传输应答信号过程中不应执行此指令
#define NRF_CMD_REUSE_TX_PAYLOAD        0xE3  //数据重发,重新使用上一包数据,CE为高,数据包被不断发送
#define NRF_CMD_READ_RX_PAYLOAD_WID     0x60  //读RX FIFO顶层R_RX_PLOAD载荷长度,若超出32字节则清除RX FIFO
#define NRF_CMD_WRITE_ACK_PAYLOAD       0xA8  //RX模式下,将PIPEppp(ppp的范围为000~101)中的AcK包与载荷一起写入,最多可以有3个挂起的ACK包.
#define NRF_CMD_WRITE_TX_PAYLOAD_NOACK  0xB0  //发送数据,但禁止自动应答,发射模式用
#define NRF_CMD_NOP                     0xFF  //空操作,可以用来读状态寄存器

#define NRF_REG_CONFIG                  0x00  //配置收发状态,CRC校验模式以及收发状态响应方式
#define NRF_REG_EN_AA                   0x01  //自动应答功能设置
#define NRF_REG_EN_RXADDR               0x02  //可用信道设置
#define NRF_REG_SETUP_AW                0x03  //收发地址宽度设置
#define NRF_REG_SETUP_RETR              0x04  //自动重发功能设置
#define NRF_REG_RF_CH                   0x05  //工作频率设置
#define NRF_REG_RF_SETUP                0x06  //发射速率、功耗功能设置0b0010 0111
#define NRF_REG_STATUS                  0x07  //状态寄存器（写1清除对应的中断）
#define NRF_REG_OBSERVE_TX              0x08  //发送监测功能
#define NRF_REG_CD                      0x09  //地址检测
#define NRF_REG_RX_ADDR_P0              0x0A  //频道0接收数据地址（通道0的地址 注： 位39到位0可以随意改 ）
#define NRF_REG_RX_ADDR_P1              0x0B  //频道1接收数据地址（通道1的地址 注： 位39到位0可以随意改 ）
#define NRF_REG_RX_ADDR_P2              0x0C  //频道2接收数据地址（通道2的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P3              0x0D  //频道3接收数据地址（通道3的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P4              0x0E  //频道4接收数据地址（通道4的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P5              0x0F  //频道5接收数据地址（通道5的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_TX_ADDR                 0x10  //发送地址寄存器（发射地址，注： 位39到位0可以随意改）
#define NRF_REG_RX_PW_P0                0x11  //接收数据通道0有效数据宽度
#define NRF_REG_RX_PW_P1                0x12  //接收数据通道1有效数据宽度
#define NRF_REG_RX_PW_P2                0x13  //接收数据通道2有效数据宽度
#define NRF_REG_RX_PW_P3                0x14  //接收数据通道3有效数据宽度
#define NRF_REG_RX_PW_P4                0x15  //接收数据通道4有效数据宽度
#define NRF_REG_RX_PW_P5                0x16  //接收数据通道5有效数据宽度
#define NRF_REG_FIFO_STATUS             0x17  //FIFO栈入栈出状态寄存器设置（只读）
#define NRF_REG_DYNPD                   0x1C  //nrf24l01+ 动态数据长度寄存器
#define NRF_REG_FEATURE                 0x1D  //nrf24l01+ 功能寄存器

#define NRF_WID_RX_ADR             5        //5字节的地址宽度
#define NRF_WID_TX_ADR             5        //5字节的地址宽度
#define NRF_WID_RX_PLOAD           32       //32字节的用户数据宽度
#define NRF_WID_ACK_PAYLOAD        5        //5字节的用户数据宽度


#endif // __NRF_LIB_H__
