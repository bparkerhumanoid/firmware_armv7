/**
\file AX58200_Hw.c
\brief Implementation

\version 1.0.1.0
*/

/* INCLUDE FILE DECLARATIONS */
#include "M480.h"

//#include "ecat_def.h"
//#include "ecatappl.h"
#include "AX58200_Hw.h"
//#include "core_cmFunc.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* NAMING CONSTANT DECLARATIONS */

/* MACRO DECLARATIONS */
#if (INTERRUPTS_SUPPORTED)
uint8_t hw_BASEPRI_value = 0;
/*-----------------------------------------------------------------------------------------
------
------    Global Interrupt setting
------
-----------------------------------------------------------------------------------------*/
#define DISABLE_GLOBAL_INT      do {__set_PRIMASK(1);} while (0)
#define ENABLE_GLOBAL_INT       do {__set_PRIMASK(0);} while (0)
#define DISABLE_AL_EVENT_INT    do {__set_BASEPRI(hw_BASEPRI_value);} while (0)
#define ENABLE_AL_EVENT_INT     do {__set_BASEPRI(0);} while (0)
/*-----------------------------------------------------------------------------------------
------
------    SYNC0 Interrupt
------
-----------------------------------------------------------------------------------------*/
#if (DC_SUPPORTED)
#define DISABLE_SYNC0_INT
#define ENABLE_SYNC0_INT
#define DISABLE_SYNC1_INT
#define ENABLE_SYNC1_INT
#endif //#if (DC_SUPPORTED)
#endif //#if (INTERRUPTS_SUPPORTED)

/* TYPE DECLARATIONS */

/* GLOBAL VARIABLES DECLARATIONS */
HW_DEBUG		HW_Debug = {0};
HW_SPI_OBJECT	HW_SpiObj;

/* LOCAL VARIABLES DECLARATIONS */

//Contains the content of the ALEvent register (0x220), this variable is updated on each Access to the Esc
UALEVENT		EscALEvent;

/* LOCAL SUBPROGRAM DECLARATIONS */
static INT32 HW_SPI_Init(void);
static void HW_SPI_Read(HW_SPI_OBJECT* pSpiObj, uint8_t *pBuf, uint16_t Addr, uint16_t ByteLen);
static void HW_SPI_Write(HW_SPI_OBJECT* pSpiObj, uint8_t *pData, uint16_t Addr, uint16_t ByteLen);
static INT32 HW_CheckVendorProductID(void);

/* LOCAL SUBPROGRAM BODIES */
/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_MultiFuncPins()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
INT32 HW_MultiFuncPins(GPIO_T* Instance, uint32_t Pins, uint8_t MultiFuncValue)
{
	uint32_t	i, bitMask=0x1ul, funcMask=0xful, mfunc=(MultiFuncValue & 0xful);
	uint32_t	*pGPx_MFPL=0, *pGPx_MFPH=0;;
	
	if (Instance == PA)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPA_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPA_MFPH;
	}
	else if (Instance == PB)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPB_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPB_MFPH;
	}
	else if (Instance == PC)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPC_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPC_MFPH;	
	}	
	else if (Instance == PD)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPD_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPD_MFPH;		
	}	
	else if (Instance == PE)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPE_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPE_MFPH;		
	}	
	else if (Instance == PF)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPF_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPF_MFPH;	
	}	
	else if (Instance == PG)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPG_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPG_MFPH;		
	}	
	else if (Instance == PH)
	{
		pGPx_MFPL = (uint32_t*)&SYS->GPH_MFPL;
		pGPx_MFPH = (uint32_t*)&SYS->GPH_MFPH;		
	}	
	else
	{
		/* Unsupported port number */
		DBG_PRINT("Unsupported port.(0x%lx)\r\n", (uint32_t)Instance);
		return -1;
	}
	
	for (i=0; i<GPIO_PIN_MAX; i++)
	{
		if (Pins & bitMask)
		{
			if (i<8)
			{
				/* Clear multi-function setting in MFPL reg. */
				(*pGPx_MFPL) &= ~funcMask;

				/* Config multi-function in MFPL reg. */
				(*pGPx_MFPL) |= mfunc;					
			}
			else
			{
				/* Clear multi-function setting in MFPH reg. */
				(*pGPx_MFPH) &= ~funcMask;

				/* Config multi-function in MFPH reg. */
				(*pGPx_MFPH) |= mfunc;		
			}
		}
		bitMask <<= 1;
		if (i == 7)
		{
			funcMask = 0xful;
			mfunc = MultiFuncValue;
		}
		else
		{
			funcMask <<= 4;
			mfunc <<= 4;
		}
	}	
	return 0;
	
} /* End of HW_MultiFuncPins() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_GPIO_WritePin()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void HW_GPIO_WritePin(GPIO_T* Instance, uint32_t Pin, uint8_t NewState)
{
	uint32_t port = (((uint32_t)Instance)&0x03C0UL) >> 6;
	uint32_t pin_num = HW_BitToNum(Pin);
	GPIO_PIN_DATA(port, pin_num) = NewState ? 1:0;
} /* End of HW_GPIO_WritePin() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_GPIO_TogglePin()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void HW_GPIO_TogglePin(GPIO_T* Instance, uint32_t Pin)
{
	uint32_t port = (((uint32_t)Instance)&0x03C0UL) >> 6;
	uint32_t pin_num = HW_BitToNum(Pin);
	GPIO_PIN_DATA(port, pin_num) ^= 1;
} /* End of HW_GPIO_TogglePin() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_GPIO_ReadPin()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint8_t HW_GPIO_ReadPin(GPIO_T* Instance, uint32_t Pin)
{
	return (((Instance)->PIN & Pin) ? 1 : 0);
} /* End of HW_GPIO_ReadPin() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_TMR_ClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void HW_TMR_ClkSource(TIMER_T* Instance, uint32_t NewState)
{
	if (Instance == TIMER0)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(TMR0_MODULE);
			CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);			
		}
		else
		{
			CLK_DisableModuleClock(TMR0_MODULE);
		}		
	}
	else if (Instance == TIMER1)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(TMR1_MODULE);
			CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, 0);			
		}
		else
		{
			CLK_DisableModuleClock(TMR1_MODULE);
		}		
	}
	else if (Instance == TIMER2)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(TMR2_MODULE);
			CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HXT, 0);			
		}
		else
		{
			CLK_DisableModuleClock(TMR2_MODULE);
		}		
	}
	else if (Instance == TIMER3)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(TMR3_MODULE);
			CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HXT, 0);			
		}
		else
		{
			CLK_DisableModuleClock(TMR3_MODULE);
		}		
	}
	else
	{
		DBG_PRINT("Unsupported TIMER clock source.(0x%lx)\r\n", (uint32_t)Instance);
		return;
	}
	
} /* End of HW_TIM_ClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_SPI_ClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void HW_SPI_ClkSource(SPI_T* Instance, uint8_t NewState)
{
	if (Instance == SPI0)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(SPI0_MODULE);			
			CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
		}
		else
		{
			CLK_DisableModuleClock(SPI0_MODULE);
		}		
	}
	else if (Instance == SPI1)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(SPI1_MODULE);			
			CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);
		}
		else
		{
			CLK_DisableModuleClock(SPI1_MODULE);
		}		
	}
	else if (Instance == SPI2)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(SPI2_MODULE);			
			CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PCLK1, MODULE_NoMsk);
		}
		else
		{
			CLK_DisableModuleClock(SPI2_MODULE);
		}		
	}
	else if (Instance == SPI3)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(SPI3_MODULE);			
			CLK_SetModuleClock(SPI3_MODULE, CLK_CLKSEL2_SPI3SEL_PCLK0, MODULE_NoMsk);
		}
		else
		{
			CLK_DisableModuleClock(SPI3_MODULE);
		}
	}	
	else
	{
		DBG_PRINT("Unsupported SPI clock source.(0x%lx)\r\n", (uint32_t)Instance);
		return;
	}
	
} /* End of HW_SPI_ClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_UART_ClkSource()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
void HW_UART_ClkSource(UART_T* Instance, uint8_t NewState)
{
	if (Instance == UART0)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(UART0_MODULE);			
			CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
		}
		else
		{
			CLK_DisableModuleClock(UART0_MODULE);
		}		
	}
	else if (Instance == UART1)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(UART1_MODULE);			
			CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));
		}
		else
		{
			CLK_DisableModuleClock(UART1_MODULE);
		}		
	}
	else if (Instance == UART2)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(UART2_MODULE);			
			CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HXT, CLK_CLKDIV4_UART2(1));
		}
		else
		{
			CLK_DisableModuleClock(UART2_MODULE);
		}		
	}
	else if (Instance == UART3)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(UART3_MODULE);			
			CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_HXT, CLK_CLKDIV4_UART3(1));
		}
		else
		{
			CLK_DisableModuleClock(UART3_MODULE);
		}		
	}
	else if (Instance == UART4)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(UART4_MODULE);			
			CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_HXT, CLK_CLKDIV4_UART3(1));
		}
		else
		{
			CLK_DisableModuleClock(UART4_MODULE);
		}		
	}
	else if (Instance == UART5)
	{
		if (NewState == ENABLE)
		{
			CLK_EnableModuleClock(UART5_MODULE);			
			CLK_SetModuleClock(UART5_MODULE, CLK_CLKSEL3_UART5SEL_HXT, CLK_CLKDIV4_UART5(1));
		}
		else
		{
			CLK_DisableModuleClock(UART5_MODULE);
		}		
	}
	else
	{
		DBG_PRINT("Unsupported UART/USART clock source.(0x%lx)\r\n", (uint32_t)Instance);
		return;
	}
	
} /* End of HW_UART_ClkSource() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_BitToNum()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint32_t HW_BitToNum(uint32_t Bit)
{
	if (Bit & 0x000000ff)
	{
		switch (Bit)
		{
		case BIT_0: return 0;
		case BIT_1: return 1;
		case BIT_2: return 2;		
		case BIT_3: return 3;				
		case BIT_4: return 4;						
		case BIT_5: return 5;						
		case BIT_6: return 6;		
		case BIT_7: return 7;				
		}
	}
	else if (Bit & 0x0000ff00)
	{
		switch(Bit)
		{
		case BIT_8: return 8;
		case BIT_9: return 9;
		case BIT_10: return 10;
		case BIT_11: return 11;
		case BIT_12: return 12;
		case BIT_13: return 13;
		case BIT_14: return 14;
		case BIT_15: return 15;
		}
	}	
	else if (Bit & 0x00ff0000)
	{	
		switch(Bit)
		{
		case BIT_16: return 16;
		case BIT_17: return 17;
		case BIT_18: return 18;
		case BIT_19: return 19;
		case BIT_20: return 20;
		case BIT_21: return 21;
		case BIT_22: return 22;
		case BIT_23: return 23;
		}
	}
	else
	{	
		switch(Bit)
		{
		case BIT_24: return 24;
		case BIT_25: return 25;
		case BIT_26: return 26;
		case BIT_27: return 27;
		case BIT_28: return 28;
		case BIT_29: return 29;
		case BIT_30: return 30;
		case BIT_31: return 31;
		}
	}
	return 0;
} /* End of HW_BitToNum() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_SPI_Init()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static INT32 HW_SPI_Init(void)
{
	/* Enable SPI clock source */
	HW_SPI_ClkSource(HW_SPI_ESC_INSTANCE, ENABLE);

	/* SPI ESC CS GPIO pin configuration  */
	HW_MultiFuncPins(HW_SPI_ESC_CS_PORT, HW_SPI_ESC_CS_PIN, 0);		
	GPIO_SetMode(HW_SPI_ESC_CS_PORT, HW_SPI_ESC_CS_PIN, GPIO_MODE_OUTPUT);
	GPIO_SetSlewCtl(HW_SPI_ESC_CS_PORT, HW_SPI_ESC_CS_PIN, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(HW_SPI_ESC_CS_PORT, HW_SPI_ESC_CS_PIN, GPIO_PUSEL_PULL_UP);
	
	/* SPI CLK GPIO pin configuration  */
	HW_MultiFuncPins(HW_SPI_ESC_SCLK_PORT, HW_SPI_ESC_SCLK_PIN, HW_SPI_ESC_MULTI_FUNC);	
	GPIO_SetSlewCtl(HW_SPI_ESC_SCLK_PORT, HW_SPI_ESC_SCLK_PIN, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(HW_SPI_ESC_SCLK_PORT, HW_SPI_ESC_SCLK_PIN, GPIO_PUSEL_PULL_UP);
	
	/* Enable clock pin schmitt trigger */	
	HW_SPI_ESC_SCLK_PORT->SMTEN |= HW_SPI_ESC_SCLK_PIN;
	
	/* SPI MISO GPIO pin configuration  */
	HW_MultiFuncPins(HW_SPI_ESC_MISO_PORT, HW_SPI_ESC_MISO_PIN, HW_SPI_ESC_MULTI_FUNC);
	GPIO_SetSlewCtl(HW_SPI_ESC_MISO_PORT, HW_SPI_ESC_MISO_PIN, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(HW_SPI_ESC_MISO_PORT, HW_SPI_ESC_MISO_PIN, GPIO_PUSEL_DISABLE);

	/* SPI MOSI GPIO pin configuration  */
	HW_MultiFuncPins(HW_SPI_ESC_MOSI_PORT, HW_SPI_ESC_MOSI_PIN, HW_SPI_ESC_MULTI_FUNC);	
	GPIO_SetSlewCtl(HW_SPI_ESC_MOSI_PORT, HW_SPI_ESC_MOSI_PIN, GPIO_SLEWCTL_FAST);
	GPIO_SetPullCtl(HW_SPI_ESC_MOSI_PORT, HW_SPI_ESC_MOSI_PIN, GPIO_PUSEL_PULL_UP);

	/* Using software controlled slave selection */
	SPI_DisableAutoSS(HW_SPI_ESC_INSTANCE);
	
	/* Clear Rx/Tx FIFO */
	SPI_ClearRxFIFO(HW_SPI_ESC_INSTANCE);
	SPI_ClearTxFIFO(HW_SPI_ESC_INSTANCE);
	SPI_SetFIFO(HW_SPI_ESC_INSTANCE, 0, 0);

	/* Enable SPI PDI */
	SPI_Open(HW_SPI_ESC_INSTANCE, SPI_MASTER, SPI_MODE_3, 8, HW_SPI_ESC_BAUDRATE);	
	
	/* Initialize CS pin(s) as idle state */
	HW_GPIO_WritePin(HW_SPI_ESC_CS_PORT, HW_SPI_ESC_CS_PIN, 1);
	
	return 0;
	
} /* End of HW_SPI_Init() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_SPI_WaitFlagStateUntilTimeout()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static INT32 HW_SPI_WaitFlagStateUntilTimeout(HW_SPI_OBJECT* pSpiObj, uint32_t Flag, uint32_t State, uint32_t Timeout, uint32_t Tickstart)
{
	SPI_T* pInst = pSpiObj->pInst;
	
	while((((pInst->STATUS & Flag) == (Flag)) ? 1 : 0) != State)
	{
		if(HW_CheckTimeout(Tickstart, Timeout))
		{
			SPI_DisableInt(pInst, SPI_FIFO_TXTH_INT_MASK|SPI_FIFO_RXTH_INT_MASK|SPI_SLVBE_INT_MASK);				
			SPI_DISABLE(pInst);
			DBG_PRINT("SPI wait flag timeout!\r\n");
			return -1;
		}
	}

	return 0;
	
} /* End of HW_SPI_WaitFlagStateUntilTimeout() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_SPI_TransmitReceive()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static INT32 HW_SPI_TransmitReceive(HW_SPI_OBJECT* pSpiObj, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
	SPI_T *pInst = pSpiObj->pInst;
	uint32_t tickstart = 0U, TxXferCount=0, RxXferCount=0;
	INT16 errorcode = 0;
	uint32_t port = (((uint32_t)pSpiObj->pCsPort)&0x03C0UL) >> 6;
	uint32_t pin_num = HW_BitToNum(pSpiObj->CsPin);

	/* Init tickstart for timeout management*/
	tickstart = HW_GetTimer();
		  
	TxXferCount = Size;
	RxXferCount = Size;
	
	/* Check if the SPI is already enabled */
	if ((pInst->CTL & SPI_CTL_SPIEN_Msk) == 0)
	{
		/* Enable SPI peripheral */
		SPI_ENABLE(pInst);
	}

	/* CS = low */
	GPIO_PIN_DATA(port, pin_num) = 0;

	/* Transmit and Receive data in 8 Bit mode */
	if(TxXferCount == 0x01)
	{
		*((__IO uint8_t*)&(pInst->TX)) = (*pTxData);
		pTxData += sizeof(uint8_t);
		TxXferCount--;
	}
	
	while((TxXferCount > 0U) || (RxXferCount > 0U))
	{
		/* check TXE flag */
		if((TxXferCount > 0U) && (pInst->STATUS & SPI_STATUS_TXEMPTY_Msk))
		{
			*(__IO uint8_t *)&(pInst->TX) = (*pTxData++);
			TxXferCount--;
		}

		/* Wait until RXNE flag is reset */
		if((RxXferCount > 0U) && (pInst->STATUS & SPI_STATUS_RXEMPTY_Msk)==0)
		{
			(*(uint8_t *)pRxData++) = pInst->RX;
			RxXferCount--;
		}

		if(HW_CheckTimeout(tickstart, Timeout))
		{
			DBG_PRINT("SPI xfer timeout!(%04x->%04x > %04x)\r\n", (uint16_t)tickstart, HW_GetTimer(), (uint16_t)Timeout);
			errorcode = -2;
			goto HW_SPI_XferError;
		}
	}

	/* Wait until TXE flag */
	if(HW_SPI_WaitFlagStateUntilTimeout(pSpiObj, SPI_STATUS_TXEMPTY_Msk, 1, Timeout, tickstart) < 0)
	{
		DBG_PRINT("SPI wait TXE timeout!\r\n");		
		errorcode = -3;
		goto HW_SPI_XferError;
	}
  
	/* Check Busy flag */
	if(HW_SPI_WaitFlagStateUntilTimeout(pSpiObj, SPI_STATUS_BUSY_Msk, 0, Timeout, tickstart) < 0)
	{
		DBG_PRINT("SPI wait BSY timeout!\r\n");
		errorcode = -4;
		goto HW_SPI_XferError;
	}

	/* Clear overrun flag in 2 Lines communication mode because received is not read */
	pInst->STATUS = SPI_STATUS_RXOVIF_Msk; 
	pInst->STATUS = SPI_STATUS_RXTOIF_Msk;

HW_SPI_XferError:
	/* CS = high */	
	GPIO_PIN_DATA(port, pin_num) = 1;
	return errorcode;

} /* End of HW_SPI_TransmitReceive() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_SPI_Read()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void HW_SPI_Read(HW_SPI_OBJECT* pSpiObj, uint8_t *pBuf, uint16_t Addr, uint16_t ByteLen)
{
	uint16_t XferLen, AddrTmp, i;
	uint8_t *ptr, DataOffset = 0;
	INT32 ret;
	
	/* Re-entry transfer detection */	
	if (pSpiObj->Lock)
	{
		pSpiObj->ReentryCnt++;
		return;
	}
	/* Lock SPI transfer section */
	pSpiObj->Lock = 1;
	
	while (ByteLen)
	{
		AddrTmp = Addr;
		
		/* Fragmented to specified data length per-section */
		if (ByteLen > HW_SpiObj.MaxFragSize)
		{
			XferLen = HW_SpiObj.MaxFragSize;
		}
		else
		{
			XferLen = ByteLen;
		}

		/* Set address, command */
		ptr = (uint8_t*)&AddrTmp;
		if (pSpiObj->ThreeByteAddrMode)
		{
			pSpiObj->TxBuf[2] = (ptr[1] & 0xe0) | (HW_SPI_READ_WITH_WAIT_CMD << 2);
			AddrTmp = AddrTmp << 3;
			pSpiObj->TxBuf[0] = ptr[1];
			pSpiObj->TxBuf[1] = ptr[0] | HW_SPI_ADDR_EXT_CMD;
			DataOffset = 3;
		}
		else
		{
			AddrTmp = AddrTmp << 3;
			pSpiObj->TxBuf[0] = ptr[1];
			pSpiObj->TxBuf[1] = ptr[0] | HW_SPI_READ_WITH_WAIT_CMD;
			DataOffset = 2;
		}
		
		/* Set dummy byte */
		pSpiObj->TxBuf[DataOffset] = 0xff;
			
		/* Set read terminal byte */
		memset(&pSpiObj->TxBuf[DataOffset+1], 0, XferLen);
		pSpiObj->TxBuf[DataOffset + XferLen] = 0xff;
		DataOffset++;

		ret = HW_SPI_TransmitReceive(pSpiObj, pSpiObj->TxBuf, pSpiObj->RxBuf, DataOffset + XferLen, HW_SPI_XFER_TIMEOUT);
		/* Start read */		
		if (ret != 0)		
		{
			break;
		}
		
		/* Store received data */
		for (i=0; i<XferLen; i++)
		{
			pBuf[i] = pSpiObj->RxBuf[DataOffset + i];
		}
		
		/* Next section */
		Addr += XferLen;
		pBuf += XferLen;
		ByteLen -= XferLen;
	}

	/* Unlock SPI transfer section */
	pSpiObj->Lock = 0;	
	return;
	
} /* End of HW_SPI_Read() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_SPI_Write()
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
static void HW_SPI_Write(HW_SPI_OBJECT* pSpiObj, uint8_t *pData, uint16_t Addr, uint16_t ByteLen)
{
	uint16_t XferLen, AddrTmp;
	uint8_t *ptr, DataOffset;
	INT32 ret;
	
	/* Re-entry transfer detection */	
	if (pSpiObj->Lock)
	{
		pSpiObj->ReentryCnt++;
		return;
	}
	/* Lock SPI transfer section */
	pSpiObj->Lock = 1;

	while (ByteLen)
	{
		AddrTmp = Addr;

		/* Fragmented to specified data length per-section */
		if (ByteLen > HW_SpiObj.MaxFragSize)
		{
			XferLen = HW_SpiObj.MaxFragSize;
		}
		else
		{
			XferLen = ByteLen;
		}
		
		/* Set address , command */
		ptr = (uint8_t*)&AddrTmp;
		if (pSpiObj->ThreeByteAddrMode)
		{		
			pSpiObj->TxBuf[2] = (ptr[1] & 0xe0) | (HW_SPI_WRITE_CMD << 2);		
			AddrTmp = AddrTmp << 3;
			pSpiObj->TxBuf[0] = ptr[1];
			pSpiObj->TxBuf[1] = ptr[0] | HW_SPI_ADDR_EXT_CMD;			
			DataOffset = 3;
		}
		else
		{
			AddrTmp = AddrTmp << 3;
			pSpiObj->TxBuf[0] = ptr[1];
			pSpiObj->TxBuf[1] = ptr[0] | HW_SPI_WRITE_CMD;			
			DataOffset = 2;			
		}
		
		/* Set transmit data bytes */
		memcpy(&(pSpiObj->TxBuf[DataOffset]), pData, XferLen);
		DataOffset++;
		
		/* Start write */
		ret = HW_SPI_TransmitReceive(pSpiObj, pSpiObj->TxBuf, pSpiObj->RxBuf, (DataOffset - 1) + XferLen, HW_SPI_XFER_TIMEOUT);
		if (ret != 0)
		{
			break;
		}
		
		Addr += XferLen;
		pData += XferLen;	
		ByteLen -= XferLen;
	}
	
	/* Unlock SPI transfer section */
	pSpiObj->Lock = 0;	
	return;
	
} /* End of HW_SPI_Write() */

#define ESC_AL_EVENT_OFFSET 0
#define ESC_AL_EVENTMASK_OFFSET 0

/**
  * @brief  The function operates a SPI access without addressing. 
  * @note   The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
  *         It will be saved in the global "EscALEvent"
  * @param  None
  * @retval None
  */
static void GetInterruptRegister(void)
{
//#if (AL_EVENT_ENABLED)
//	DISABLE_AL_EVENT_INT;
//#endif //#if (AL_EVENT_ENABLED)

	HW_EscRead((MEM_ADDR*)EscALEvent.Byte, ESC_AL_EVENT_OFFSET, 2);
	
//#if (AL_EVENT_ENABLED)
//	ENABLE_AL_EVENT_INT;
//#endif ////#if (AL_EVENT_ENABLED)
} /* End of GetInterruptRegister() */

/**
  * @brief  The function operates a SPI access without addressing.
  * @note   Shall be implemented if interrupts are supported else this function is equal to "GetInterruptRegsiter()"
  * @note   The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
  *         It will be saved in the global "EscALEvent"
  * @param  None
  * @retval None
  */
#if (INTERRUPTS_SUPPORTED)
static void ISR_GetInterruptRegister(void)
{
	HW_EscReadIsr((MEM_ADDR*)EscALEvent.Byte, ESC_AL_EVENT_OFFSET, 2);
	
} /* End of ISR_GetInterruptRegister() */
#else // !(INTERRUPTS_SUPPORTED)
#define ISR_GetInterruptRegister GetInterruptRegister
#endif //#if (INTERRUPTS_SUPPORTED)

/* EXPORTED SUBPROGRAM BODIES */
#if (ESC_PHY_ENHANCEMENT)
/**
  * @brief  This function operates to read PHY register.
  * @param  PhyAddr       PHY address.
  * @param  RegAddr       Address of the PHY register.
  * @param  *pBuf         Point to the empty buffer will be stored data.
  * @param  Size          Byte size that will be read.
  * @retval Operation result, return zero indicates the operation is successful, otherwise return a error code.
  */
INT32 HW_MiiPhyRead(uint8_t PhyAddr, uint8_t RegAddr, uint16_t *pBuf, uint8_t Size)
{
	uint8_t	tmp8;
	uint16_t tmp16;
	
	/* Check access right */
	HW_EscRead((MEM_ADDR *)&tmp16, 0x0510, 2);
	if ((tmp16 & 0x8002) == 0)
	{
		return -1;
	}
	
	/* Get access right to PDI */
	tmp8 = 1;
	HW_EscWrite((MEM_ADDR *)&tmp8, 0x0517, 1);
	
	/* Set PHY address */
	HW_EscWrite((MEM_ADDR *)&PhyAddr, 0x0512, 1);
	
	while (Size--)
	{
		/* Set register address */
		HW_EscWrite((MEM_ADDR *)&RegAddr, 0x0513, 1);

		/* Set command */
		tmp16 |= 0x0100;
		tmp16 &= ~0x2000;
		HW_EscWrite((MEM_ADDR *)&tmp16, 0x0510, 2);
	
		/* Check complete */
		do {
			HW_EscRead((MEM_ADDR *)&tmp16, 0x0510, 2);
		} while (tmp16 & 0x8300);
	
		/* Check error */
		if (tmp16 & 0x6000)
		{
			return -2;
		}
	
		/* Read data */
		HW_EscRead((MEM_ADDR *)pBuf, 0x0514, 2);
		
		RegAddr++;
		pBuf++;
	}
	
	return 0;
}

/**
  * @brief  This function operates to write PHY register.
  * @param  PhyAddr       PHY address.
  * @param  RegAddr       Address of the PHY register.
  * @param  *pData        Point to the data buffer will be written.
  * @param  Len           Byte size that will be written.
  * @retval Operation result, return zero indicates the operation is successful, otherwise return a error code.
  */
INT32 HW_MiiPhyWrite(uint8_t PhyAddr, uint8_t RegAddr, uint16_t *pData, uint8_t Len)
{
	uint8_t	tmp8;
	uint16_t tmp16;
	
	/* Check access right */
	HW_EscRead((MEM_ADDR *)&tmp16, 0x0510, 2);
	if ((tmp16 & 0x8002) == 0)
	{
		return -1;
	}
	
	/* Get access right to PDI */
	tmp8 = 1;
	HW_EscWrite((MEM_ADDR *)&tmp8, 0x0517, 1);
	
	/* Set PHY address */
	HW_EscWrite((MEM_ADDR *)&PhyAddr, 0x0512, 1);
	
	while (Len--)
	{
		/* Set register address */
		HW_EscWrite((MEM_ADDR *)&RegAddr, 0x0513, 1);
	
		/* Write data */
		HW_EscWrite((MEM_ADDR *)pData, 0x0514, 2);
		
		/* Set command */
		tmp16 |= 0x0201;
		HW_EscWrite((MEM_ADDR *)&tmp16, 0x0510, 2);
	
		/* Check complete */
		do {
			HW_EscRead((MEM_ADDR *)&tmp16, 0x0510, 2);
		} while (tmp16 & 0x8300);
	
		/* Check error */
		if (tmp16 & 0x4000)
		{
			return -2;
		}
	
		RegAddr++;
		pData++;
	}
	
	return 0;
}

/**
  * @brief  This function operates to do PHY enhancement.
  */
void HW_PhyEnhancement(void)
{
	uint8_t i;
	uint16_t tmp16;
	static uint16_t bmsr[2][2] = {{0, 0}, {0, 0}};

	HW_DEBUG *pDbgCnt;
	pDbgCnt = HW_GetDebugCounter();
	
	for (i=0; i<2; i++)
	{
		/* Set page 0 */
		tmp16 = 0;
		HW_MiiPhyWrite(i, 0x1f, &tmp16, 1);
		
		HW_MiiPhyRead(i, 0x01, &(bmsr[i][0]), 1);
		if ((bmsr[i][1] & 0x0020)==0 && (bmsr[i][0] & 0x0020)!=0)
		{
			HW_MiiPhyRead(i, 0x1c, &tmp16, 1);
			
			if (tmp16)
			{
				HW_MiiPhyRead(i, 0x1e, &tmp16, 1);
				tmp16 |= 0x4000;
				HW_MiiPhyWrite(i, 0x1e, &tmp16, 1);
				
				HW_MiiPhyRead(i, 0x00, &tmp16, 1);
				tmp16 |= 0x1200;
				HW_MiiPhyWrite(i, 0x00, &tmp16, 1);	

				pDbgCnt->PhyEnhancementCnt[i]++;
			}
		}
		bmsr[i][1] = bmsr[i][0];
	}
	
}
#endif

/**
  * @brief  This function intialize the Process Data Interface (PDI) and the host controller.
  * @param  None
  * @retval 0 if initialization was successful
  */
uint8_t HW_Init(void)
{
	uint32_t	intMask;
#if (ESC_EEPDONE_CHECK_ENABLE)	
	uint16_t	tmp16;
#endif

	/* Calculate BASEPRI value will be masking */
#if (INTERRUPTS_SUPPORTED)
	hw_BASEPRI_value = NVIC_EncodePriority(HW_INT_PRIORITY_GROUP, HW_SYNCx_INT_PREEMPTION_PRIORITY, HW_SYNCx_INT_SUB_PRIORITY);
	hw_BASEPRI_value <<= 4;
#endif

	/* Intialize HW debug data structure */
	memset(&HW_Debug, 0, sizeof(HW_Debug));	

	/* Initialize PDIs */
	HW_SPI_Init();	
	memset(&HW_SpiObj, 0, sizeof(HW_SPI_OBJECT));
	HW_SpiObj.pInst = HW_SPI_ESC_INSTANCE;
	HW_SpiObj.pCsPort = HW_SPI_ESC_CS_PORT;
	HW_SpiObj.CsPin = HW_SPI_ESC_CS_PIN;
	HW_SpiObj.ThreeByteAddrMode = 1;
	HW_SpiObj.MaxFragSize = HW_SPI_MAX_DATA_FRAGMENT_SIZE;
	
	/* Enable time tick peripheral */
	HW_TMR_ClkSource(HW_TIMETICK_INSTANCE, ENABLE);
	TIMER_Open(HW_TIMETICK_INSTANCE, TIMER_CONTINUOUS_MODE, 1000);
	TIMER_SET_PRESCALE_VALUE(HW_TIMETICK_INSTANCE, HW_TIMETICK_PRESCALER);
	HW_TIMETICK_INSTANCE->CMP = HW_TIMETICK_MASK;

	/* Start time tick */
	TIMER_Start(HW_TIMETICK_INSTANCE);

#if (ESC_EEPDONE_CHECK_ENABLE)
	/* Enable EEPROM done GPIO */
	HW_MultiFuncPins(HW_EEPROM_PORT, HW_EEPROM_PIN, 0);
	GPIO_SetMode(HW_EEPROM_PORT, HW_EEPROM_PIN, GPIO_MODE_INPUT);
	GPIO_SetSlewCtl(HW_EEPROM_PORT, HW_EEPROM_PIN, GPIO_SLEWCTL_HIGH);
	GPIO_SetPullCtl(HW_EEPROM_PORT, HW_EEPROM_PIN, GPIO_PUSEL_PULL_DOWN);

	/* Wait ESC ready by checking EEPDONE pin goes high */
	tmp16 = HW_GetTimer();
	do
	{
		if (HW_CheckTimeout(tmp16, HW_EEPROM_RELOAD_TIMEOUT))
		{
			return 1;
		}
	} while (HW_GPIO_ReadPin(HW_EEPROM_PORT, HW_EEPROM_PIN) == 0);
#endif //#if (ESC_EEPDONE_CHECK_ENABLE)

	do
	{
		intMask = 0x93;
		HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
		intMask = 0;
		HW_EscReadDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
	} while (intMask != 0x93);
	intMask = 0x00;
	HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);

	/* Check Chip ID */
	if (HW_CheckVendorProductID() < 0)
	{
		return 2;
	}

#if (AL_EVENT_ENABLED)
	/* Configure GPIO as Interrupt input */
	HW_MultiFuncPins(HW_ALEVENT_PORT, HW_ALEVENT_PIN, 0);
	GPIO_SetMode(HW_ALEVENT_PORT, HW_ALEVENT_PIN, GPIO_MODE_INPUT);
	GPIO_SetSlewCtl(HW_ALEVENT_PORT, HW_ALEVENT_PIN, GPIO_SLEWCTL_HIGH);
	GPIO_SetPullCtl(HW_ALEVENT_PORT, HW_ALEVENT_PIN, GPIO_PUSEL_PULL_UP);
	GPIO_EnableInt(HW_ALEVENT_PORT, HW_BitToNum(HW_ALEVENT_PIN), GPIO_INT_FALLING);	

	/* Enable SPI interrupt source at NVIC */
	NVIC_SetPriority(HW_ALEVENT_IRQ, NVIC_EncodePriority(HW_INT_PRIORITY_GROUP, HW_ALEVENT_INT_PREEMPTION_PRIORITY, HW_ALEVENT_INT_SUB_PRIORITY));
	NVIC_EnableIRQ(HW_ALEVENT_IRQ);
	
	/* Enable ESC interrupt */	
	ENABLE_ESC_INT();
#endif //#if (AL_EVENT_ENABLED)

#if (DC_SUPPORTED)
	/* Enable SYNC0 GPIO */
	HW_MultiFuncPins(HW_SYNC0_PORT, HW_SYNC0_PIN, 0);	
	GPIO_SetMode(HW_SYNC0_PORT, HW_SYNC0_PIN, GPIO_MODE_INPUT);
	GPIO_SetSlewCtl(HW_SYNC0_PORT, HW_SYNC0_PIN, GPIO_SLEWCTL_HIGH);
	GPIO_SetPullCtl(HW_SYNC0_PORT, HW_SYNC0_PIN, GPIO_PUSEL_PULL_UP);
	GPIO_EnableInt(HW_SYNC0_PORT, HW_BitToNum(HW_SYNC0_PIN), GPIO_INT_FALLING);

	/* Enable SYNC0 interrupt source at NVIC */	
	NVIC_SetPriority(HW_SYNC0_IRQ, NVIC_EncodePriority(HW_INT_PRIORITY_GROUP, HW_SYNCx_INT_PREEMPTION_PRIORITY, HW_SYNCx_INT_SUB_PRIORITY));
	NVIC_EnableIRQ(HW_SYNC0_IRQ);

	/* Enable SYNC1 GPIO */
	HW_MultiFuncPins(HW_SYNC1_PORT, HW_SYNC1_PIN, 0);
	GPIO_SetMode(HW_SYNC1_PORT, HW_SYNC1_PIN, GPIO_MODE_INPUT);
	GPIO_SetSlewCtl(HW_SYNC1_PORT, HW_SYNC1_PIN, GPIO_SLEWCTL_HIGH);
	GPIO_SetPullCtl(HW_SYNC1_PORT, HW_SYNC1_PIN, GPIO_PUSEL_PULL_UP);
	GPIO_EnableInt(HW_SYNC1_PORT, HW_BitToNum(HW_SYNC1_PIN), GPIO_INT_FALLING);

	/* Enable SYNC1 interrupt source at NVIC */
	NVIC_SetPriority(HW_SYNC1_IRQ, NVIC_EncodePriority(HW_INT_PRIORITY_GROUP, HW_SYNCx_INT_PREEMPTION_PRIORITY, HW_SYNCx_INT_SUB_PRIORITY));		
	NVIC_EnableIRQ(HW_SYNC1_IRQ);

	/* Enable SYNC0/SYNC1 interrupt */
	ENABLE_SYNC0_INT;
	ENABLE_SYNC1_INT;
#endif //#if (DC_SUPPORTED)

#if ECAT_TIMER_INT
	/* Enable time task peripheral */
	HW_TMR_ClkSource(HW_TIMETASK_INSTANCE, ENABLE);
	TIMER_Open(HW_TIMETASK_INSTANCE, TIMER_PERIODIC_MODE, HW_TIMETASK_FREQUENCY);
	TIMER_EnableInt(HW_TIMETASK_INSTANCE);

	/* Enable timer interrupt source at NVIC */
	NVIC_SetPriority(HW_TIMETASK_IRQ, NVIC_EncodePriority(HW_INT_PRIORITY_GROUP, HW_TIMETASK_INT_PREEMPTION_PRIORITY, HW_TIMETASK_INT_SUB_PRIORITY));
	NVIC_EnableIRQ(HW_TIMETASK_IRQ);

	/* Start timer with interrupt */
	TIMER_Start(HW_TIMETASK_INSTANCE);
#endif

#if (INTERRUPTS_SUPPORTED)
	/* enable all interrupts */
 	ENABLE_GLOBAL_INT;
#endif //#if (INTERRUPTS_SUPPORTED)

	return 0;
	
} /* End of HW_Init() */

/**
  * @brief  This function shall be implemented if hardware resources need to be release
  *         when the sample application stops.
  * @param  None
  * @retval None
  */
void HW_Release(void)
{

} /* End of HW_Release() */

/**
  * @brief  This function gets the current content of ALEvent register
  * @param  None
  * @retval First two Bytes of ALEvent register (0x220)
  */
uint16_t HW_GetALEventRegister(void)
{
    GetInterruptRegister();
    return EscALEvent.Word;
}

#if (INTERRUPTS_SUPPORTED)
/**
  * @brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
  *         The behaviour is equal to "HW_GetALEventRegister()"
  * @param  None
  * @retval First two Bytes of ALEvent register (0x220)
  */
uint16_t HW_GetALEventRegister_Isr(void)
{
	ISR_GetInterruptRegister();
	return EscALEvent.Word;
}
#endif //#if (INTERRUPTS_SUPPORTED)

/**
  * @brief  This function operates the SPI read access to the EtherCAT ASIC.
  * @param  pData     Pointer to a byte array which holds data to write or saves read data.
  * @param  Address   EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
  * @param  Len       Access size in Bytes.
  * @retval None
  */
void HW_EscRead(MEM_ADDR *pData, uint16_t Address, uint16_t Len)
{
#if (AL_EVENT_ENABLED)
	DISABLE_AL_EVENT_INT;
#endif //#if (AL_EVENT_ENABLED)
	HW_SPI_Read(&HW_SpiObj, (uint8_t*)pData, Address, Len);
		
#if (AL_EVENT_ENABLED)
	ENABLE_AL_EVENT_INT;
#endif //#if (AL_EVENT_ENABLED)
} /* End of HW_EscRead() */

#if (INTERRUPTS_SUPPORTED)
/**
  * @brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
  *         The behaviour is equal to "HW_EscRead()"
  * @param  pData       Pointer to a byte array which holds data to write or saves read data.
  * @param  Address     EtherCAT ASIC address ( upper limit is 0x1FFF ) for access.
  * @param  Len         Access size in Bytes.
  * @retval None
  */
void HW_EscReadIsr(MEM_ADDR *pData, uint16_t Address, uint16_t Len)
{
	HW_SPI_Read(&HW_SpiObj, (uint8_t*)pData, Address, Len);
}
#endif //#if (INTERRUPTS_SUPPORTED)

/**
  * @brief  This function operates the SPI write access to the EtherCAT ASIC.
  * @param  pData       Pointer to a byte array which holds data to write or saves write data.
  * @param  Address     EtherCAT ASIC address ( upper limit is 0x1FFF ) for access.
  * @param  Len         Access size in Bytes.
  * @retval None
  */
void HW_EscWrite(MEM_ADDR *pData, uint16_t Address, uint16_t Len)
{
#if (AL_EVENT_ENABLED)
	DISABLE_AL_EVENT_INT;
#endif //#if (AL_EVENT_ENABLED)
	HW_SPI_Write(&HW_SpiObj, (uint8_t*)pData, Address, Len);

#if (AL_EVENT_ENABLED)
	ENABLE_AL_EVENT_INT;
#endif //#if (AL_EVENT_ENABLED)
} /* End of HW_EscWrite() */

#if (INTERRUPTS_SUPPORTED)
/**
  * @brief  The SPI PDI requires an extra ESC write access functions from interrupts service routines.
  *         The behaviour is equal to "HW_EscWrite()"
  * @param  pData       Pointer to a byte array which holds data to write or saves write data.
  * @param  Address     EtherCAT ASIC address ( upper limit is 0x1FFF ) for access.
  * @param  Len         Access size in Bytes.
  * @retval None
  */
void HW_EscWriteIsr(MEM_ADDR *pData, uint16_t Address, uint16_t Len)
{
	HW_SPI_Write(&HW_SpiObj, (uint8_t*)pData, Address, Len);
}
#endif //#if (INTERRUPTS_SUPPORTED)

#if (BOOTSTRAPMODE_SUPPORTED)
/**
  * @brief  This function resets the hardware
  * @param  None
  * @retval None
  */
void HW_RestartTarget(void)
{
} /* End of HW_RestartTarget() */
#endif //#if (BOOTSTRAPMODE_SUPPORTED)

#if (ESC_EEPROM_EMULATION)
/**
  * @brief  This function is called when the master has request an EEPROM reload during EEPROM emulation
  * @param  None
  * @retval 0 if reload was successful
  */
uint16_t HW_EepromReload(void)
{
    return 0;
	
} /* End of HW_EepromReload() */
#endif //#if (ESC_EEPROM_EMULATION)

#if (AL_EVENT_ENABLED)
/**
  * @brief  Interrupt service routine for AL_EVENT
  * @param  None
  * @retval None
  */
void HW_ALEVENT_SYNC1_IRQHandler(void)
{
	if (GPIO_GET_INT_FLAG(HW_ALEVENT_PORT, HW_ALEVENT_PIN))
	{
		GPIO_CLR_INT_FLAG(HW_ALEVENT_PORT, HW_ALEVENT_PIN);
		PDI_Isr();
		HW_Debug.EscIsrCnt++;
	}
	
	if (GPIO_GET_INT_FLAG(HW_SYNC1_PORT, HW_SYNC1_PIN))
	{
		GPIO_CLR_INT_FLAG(HW_SYNC1_PORT, HW_SYNC1_PIN);
		Sync1_Isr();
		HW_Debug.Sync1IsrCnt++;
	}

} /* End of HW_ALEVENT_SYNC1_IRQHandler() */
#endif //#if (AL_EVENT_ENABLED)

#if (DC_SUPPORTED)
/**
  * @brief  Interrupt service routine for SYNC0/SYNC1
  * @param  None
  * @retval None
  */
void HW_SYNC0_IRQHandler(void)
{
	if (GPIO_GET_INT_FLAG(HW_SYNC0_PORT, HW_SYNC0_PIN))
	{
		GPIO_CLR_INT_FLAG(HW_SYNC0_PORT, HW_SYNC0_PIN);
		Sync0_Isr();
		HW_Debug.Sync0IsrCnt++;
	}
} /* End of HW_SYNC0_IRQHandler() */

#endif //#if (DC_SUPPORTED)

#if ECAT_TIMER_INT
/**
  * @brief  Interrupt service routine for the Timer
  * @param  None
  * @retval None
  */
void HW_TIMETASK_IRQHandler(void)
{
	if (TIMER_GetIntFlag(HW_TIMETASK_INSTANCE)!=0)
	{
		TIMER_ClearIntFlag(HW_TIMETASK_INSTANCE);
		HW_Debug.TmrTskIsrCnt++;
		ECAT_CheckTimer();		
	}
	
} /* End of HW_TIMETASK_IRQHandler() */
#endif

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_CheckTimeout
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
uint8_t HW_CheckTimeout(uint16_t StartTime, uint16_t Timeout)
{
	uint16_t tmp16 = HW_GetTimer();
	
	if (tmp16 < StartTime)
	{
		tmp16 = (HW_TIMETICK_MAX_VALUE - StartTime) + tmp16;
	}
	else
	{
		tmp16 = tmp16 - StartTime;		
	}

	return ((tmp16 >= Timeout) ? 1 : 0);
	
} /* End of HW_CheckTimeout() */


/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_CheckVendorProductID
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
INT32 HW_CheckVendorProductID(void)
{
	oIC_VENDOR_ID    vendor_id;	
	oIC_PRODUCT_ID   product_id;

	AX_INTF_EscRead(ESC_VENDOR_ID_REG, vendor_id.d32, sizeof(oIC_VENDOR_ID));
	if (vendor_id.b.vendor_id != ESC_VENDOR_ID)
	{
		return -1;
	}
	
	AX_INTF_EscRead(ESC_PRODUCT_ID_REG, product_id.d32, sizeof(oIC_PRODUCT_ID));	
	if (product_id.b.chip_revision != ESC_CHIP_REVISION)
	{
		return -2;
	}
	
	if (product_id.b.product_id != ESC_PRODUCT_ID)
	{
		return -3;
	}

	return 0;
	
} /* End of HW_CheckVendorProductID() */

/*
 * ----------------------------------------------------------------------------
 * Function Name: HW_GetDebugCounter
 * Purpose:
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
HW_DEBUG* HW_GetDebugCounter(void)
{
	return (&HW_Debug);
	
} /* End of HW_GetDebugCounter() */

/* End of AX58200_Hw.c */
/** @} */
