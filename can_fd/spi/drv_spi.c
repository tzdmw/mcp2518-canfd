/*******************************************************************************
  SPI Driver:  Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi.c

  Summary:
    Implementation of MCU specific SPI functions.

  Description:
    .
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software and 
any derivatives exclusively with Microchip products. It is your responsibility 
to comply with third party license terms applicable to your use of third party 
software (including open source software) that may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, 
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES 
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER 
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF 
THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED 
BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID 
DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
//DOM-IGNORE-END

// Include files
#include "drv_spi.h"


void can_cfg_init(CANFDSPI_MODULE_ID CANFD_CH1,CAN_BITTIME_SETUP baud)
	{
	REG_CiFLTOBJ fObj;
	REG_CiMASK mObj;
	CAN_TX_FIFO_CONFIG txConfig;
	CAN_RX_FIFO_CONFIG rxConfig;
	CAN_CONFIG config;
	// Reset device
	DRV_CANFDSPI_Reset(CANFD_CH1); //复位MCP2518FD 所有SFR和状态机都会像上电复位期间一样复位，器件会立即进入配置模式
	// Enable ECC and initialize RAM
	DRV_CANFDSPI_EccEnable(CANFD_CH1);    //使能ECC(ECC逻辑支持单个位错误纠正和双位错误检测)
	DRV_CANFDSPI_RamInit(CANFD_CH1, 0xff);         //并将RAM空间初始化为初值0xFF
	// Configure device
	DRV_CANFDSPI_ConfigureObjectReset(&config);       //MCP2518FD配置信息复位

	config.IsoCrcEnable = 1;         // 使能CAN FD帧中的ISO CRC位
	config.StoreInTEF = 0;            // 不将发送的报文保存到TEF中，也就不在RAM中预留TEF空间
	config.BitRateSwitchDisable = 0;  // Depends on the BRS bit on TX msg
	//CiCON->addr:0x00-03
	DRV_CANFDSPI_Configure(CANFD_CH1, &config);     //MCP2518FD配置
	// Setup TX FIFO  发送FIFO配置
	DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
	txConfig.FifoSize = 7;                                  // 采用FIFO2作为发送FIFO
	txConfig.PayLoadSize = CAN_PLSIZE_8;     // 有效负载大小位8个数据字节
	txConfig.TxPriority = 0;                   // 使能奇偶校验位
	//CiTXQCON->addr:0x50-53 + CAN_FIFO_CHn*12
	DRV_CANFDSPI_TransmitChannelConfigure(CANFD_CH1, CAN_FIFO_CH2, &txConfig);
	// Setup RX FIFO              接收FIFO配置
	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
	rxConfig.FifoSize = 15;                                  // 采用FIFO1作为接收FIFO
	rxConfig.PayLoadSize = CAN_PLSIZE_8;     // 有效负载大小位8个数据字节
	rxConfig.RxTimeStampEnable = 0;                 //不捕捉时间戳
	//CiFIFOCON1->addr:0x50-53 + CAN_FIFO_CHn*12
	DRV_CANFDSPI_ReceiveChannelConfigure(CANFD_CH1, CAN_FIFO_CH1, &rxConfig);
	// Setup RX Filter           接收滤波器设置 ，只接收数据侦ID为0x128的数据（前提是屏蔽寄存器有效）
	fObj.word = 0;
	fObj.bF.SID = 0x000;  // 接收标准标识符 11bit
	fObj.bF.SID11 = 0;
	fObj.bF.EXIDE = 0;   // 接收扩展标识符使能位 1 enable, 0 disable  1bit
	fObj.bF.EID = 0;   // 接收扩展标识符 18bit
	//CiFLTCON0->0x1D0-0x1D3
	DRV_CANFDSPI_FilterObjectConfigure(CANFD_CH1, CAN_FILTER0, &fObj.bF);
	// Setup RX Mask 接收屏蔽器设置 高电平有效
	mObj.word = 0;          // 32bit 寄存器写法，这里不用
	mObj.bF.MSID = 0x000;  // 接收标准标识符屏蔽位 和fObj.bF.SID一起等于0x000,接收全部 id
	mObj.bF.MSID11 = 0;
	mObj.bF.MIDE = 1;  // 只适用于扩展侦模式，标准侦模式时，这个位不起作用
	mObj.bF.MEID = 0;    // 接收扩展标识符屏蔽位
	//CiMASK0
	DRV_CANFDSPI_FilterMaskConfigure(CANFD_CH1, CAN_FILTER0, &mObj.bF);
	// Link FIFO and Filter 将接收滤波器与接收屏蔽器与接收FIFO绑定，则满足接收滤波器和接收屏蔽器规则的报文会在相应的FIFO接收。
	DRV_CANFDSPI_FilterToFifoLink(CANFD_CH1, CAN_FILTER0, CAN_FIFO_CH1, true);
	// Setup Bit Time 设置位时间  总线波特率 baud，这里采用自动测量发送器延时的方式实现二次采样点采集数据位
	// CiNBTCFG->0x04-0x07
	DRV_CANFDSPI_BitTimeConfigure(CANFD_CH1, baud, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
	// Setup Transmit and Receive Interrupts
	// IOCONN->0xE04-0xE07
	DRV_CANFDSPI_GpioModeConfigure(CANFD_CH1, GPIO_MODE_INT, GPIO_MODE_INT);
	//CiFOFICON0
	DRV_CANFDSPI_ReceiveChannelEventEnable(CANFD_CH1, CAN_FIFO_CH1, CAN_RX_FIFO_NOT_EMPTY_EVENT);
	//CiINT->0x1C
	DRV_CANFDSPI_ModuleEventEnable(CANFD_CH1, CAN_RX_EVENT);
	// Select Normal Mode
	//CiCON->0x00-0x03
	DRV_CANFDSPI_OperationModeSelect(CANFD_CH1, CAN_NORMAL_MODE);
	//     DRV_CANFDSPI_OperationModeSelect(CANFD_CH1, CAN_INTERNAL_LOOPBACK_MODE);
}


int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
	int8_t error = 0; 
 // Assert CS 
 error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, true); 
 if (error != 0) return error; 

 switch (spiSlaveDeviceIndex) 
 { 
 case DRV_CANFDSPI_INDEX_0: 
 HAL_SPI_TransmitReceive(&hspi1,SpiTxData,SpiRxData,spiTransferSize,1000); 
 break; 
 case DRV_CANFDSPI_INDEX_1: 
 HAL_SPI_TransmitReceive(&hspi1,SpiTxData,SpiRxData,spiTransferSize,1000); 
 break; 
 default: 
 break; 
 } 
 error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, false); 

 return error;
}

int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex, bool assert) 
{ 
int8_t error = 0; 

// Select Chip Select 
switch (spiSlaveDeviceIndex) { 
case DRV_CANFDSPI_INDEX_0: 
if (assert)  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 
error = 0; 
break; 
case DRV_CANFDSPI_INDEX_1: 
if (assert) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 
error = 0; 
break; 
default: 
break; 
} 
return error; 
} 

//#define USE_SPI_FUNCTIONS


void MCP2518FD_TransmitMessageQueue(CANFDSPI_MODULE_ID index, uint16_t id, uint8_t *data, CAN_DLC len)
{
	CAN_TX_FIFO_EVENT txFlags;
	CAN_TX_MSGOBJ txObj;

//    // Check if FIFO is not full
//    do {
//#ifdef APP_USE_TX_INT
//        Delay_us(50);
//#else
//        DRV_CANFDSPI_TransmitChannelEventGet(index, CAN_FIFO_CH2, &txFlags);
//#endif
//    }
//#ifdef APP_USE_TX_INT
//    while (!APP_TX_INT());
//#else
//    while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));
//#endif

    // Load message and transmit
    uint8_t n = DRV_CANFDSPI_DlcToDataBytes(len);
    txObj.bF.id.SID = id;
    txObj.bF.ctrl.DLC = len;//CAN_DLC_8
    txObj.bF.ctrl.IDE = 0;
    txObj.bF.ctrl.RTR = 0;
    txObj.bF.ctrl.BRS = 0;
    txObj.bF.ctrl.FDF = 1;

    DRV_CANFDSPI_TransmitChannelLoad(index, CAN_FIFO_CH2, &txObj, data, n, true);

}

void MCP2518FD_ReceiveMessage(CANFDSPI_MODULE_ID index, uint8_t nBytes)
{
	CAN_RX_MSGOBJ rxObj;
	CAN_RX_FIFO_EVENT rxFlags;
	uint8_t rxdata[8];

	DRV_CANFDSPI_ReceiveChannelEventGet(index, CAN_FIFO_CH1, &rxFlags);

	if(rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT)
	{
		DRV_CANFDSPI_ReceiveMessageGet(index, CAN_FIFO_CH1, &rxObj, rxdata, nBytes);
		if (nBytes == 8)
		{
			MCP2518FD_TransmitMessageQueue(0, rxObj.bF.id.SID, rxdata, nBytes);
		}
	}
}

