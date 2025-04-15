/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include "ecat_slv.h"
#include "utypes.h"
//#include "xmc_gpio.h"
#include "M480.h"

//#ifdef XMC4800_F144x2048
//#define P_LED  P5_8
//#define P_BTN  P15_12
//#endif
//
//#ifdef XMC4300_F100x256
//#define P_LED  P4_1
//#define P_BTN  P3_4
//#endif

extern void ESC_eep_handler(void);

/* Application variables */
_Rbuffer    Rb;
_Wbuffer    Wb;
_Cbuffer    Cb;

uint8_t * rxpdo = (uint8_t *)&Wb.LED;
uint8_t * txpdo = (uint8_t *)&Rb.button;

uint32_t encoder_scale;
uint32_t encoder_scale_mirror;

//static const XMC_GPIO_CONFIG_t gpio_config_btn = {
//  .mode = XMC_GPIO_MODE_INPUT_INVERTED_PULL_UP,
//  .output_level = 0,
//  .output_strength = 0
//};

//static const XMC_GPIO_CONFIG_t gpio_config_led = {
//  .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
//  .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
//  .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_SOFT_EDGE
//};

void cb_get_inputs (void)
{
   //Rb.button = XMC_GPIO_GetInput(P_BTN);
   Rb.button = 0;
   Cb.reset_counter++;
   Rb.encoder =  ESCvar.Time;
}

void cb_set_outputs (void)
{
   if (Wb.LED) {
     //XMC_GPIO_SetOutputHigh(P_LED);
   } else {
     //XMC_GPIO_SetOutputLow(P_LED);
   }
}

uint32_t post_object_download_hook (uint16_t index, uint8_t subindex,
                                uint16_t flags)
{
   switch(index)
   {
      case 0x7100:
      {
         switch (subindex)
         {
            case 0x01:
            {
               encoder_scale_mirror = encoder_scale;
               break;
            }
         }
         break;
      }
      case 0x8001:
      {
         switch (subindex)
         {
            case 0x01:
            {
               Cb.reset_counter = 0;
               break;
            }
         }
         break;
      }
   }
   return 0;
}

void SYS_Init(void);
void CONSOLE_Init(void);
uint8_t HW_Init(void);

void soes (void * arg)
{
   /* Setup config hooks */
   static esc_cfg_t config =
   {
      .user_arg = NULL,
      .use_interrupt = 0,
      .watchdog_cnt = 5000,
      .set_defaults_hook = NULL,
      .pre_state_change_hook = NULL,
      .post_state_change_hook = NULL,
      .application_hook = NULL,
      .safeoutput_override = NULL,
      .pre_object_download_hook = NULL,
      .post_object_download_hook = &post_object_download_hook,
      .rxpdo_override = NULL,
      .txpdo_override = NULL,
      .esc_hw_interrupt_enable = NULL,
      .esc_hw_interrupt_disable = NULL,
      .esc_hw_eep_handler = ESC_eep_handler
   };

   /* Unlock protected registers */	
   SYS_UnlockReg();

   /* System initialization */
   SYS_Init();

   /* Initialize console */
//   CONSOLE_Init();

#if 0
   {
     uint8_t b[64];

     while (1) {
       b[0] = 'A';
       UART_Write(UART2, b, 1);
     }
   }
#endif

   printf("\nSOES (Simple Open EtherCAT Slave)\n");

#if 0
   if (HW_Init()) {
     printf("HW_Init() failed\n");
   }
#endif

   // configure I/O
//   XMC_GPIO_Init(P_BTN, &gpio_config_btn);
//   XMC_GPIO_Init(P_LED, &gpio_config_led);

   ecat_slv_init (&config);

   while (1)
   {
      ecat_slv();
   }
}

int main (void)
{
   soes (NULL);
   return 0;
}

#define MC_INT_PRIORITY_GROUP           3

/*
 * ----------------------------------------------------------------------------
 * Function Name: void SYS_Init(void)
 * Purpose: System initialization
 * Params:	None
 * Returns:	None
 * Note:	Setup clock, multi-function register and UART
 * ----------------------------------------------------------------------------
 */
void SYS_Init(void)
{
	uint32_t volatile i;
	
	/* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
	PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

	/* Enable External XTAL (4~24 MHz) */
	CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

	/* Waiting for 12MHz clock ready */
	CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

	/* Switch HCLK clock source to HXT */
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

	/* Set core clock as PLL_CLOCK from PLL */
	CLK_SetCoreClock(FREQ_192MHZ);

	/* Set both PCLK0 and PCLK1 as HCLK/2 */
	CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

	SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;    /* select HSUSBD */
	/* Enable USB PHY */
	SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;
	for (i=0; i<0x1000; i++);      // delay > 10 us
	SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;
	
	/* Enable IP clock */
	CLK_EnableModuleClock(HSUSBD_MODULE);

	/* Enable system tick */
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(MC_INT_PRIORITY_GROUP, 3, 0));	
	CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_HXT, CLK_GetHXTFreq()/1000);//Set 1ms tick
	
	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
	SystemCoreClockUpdate();
	
	/* Enable IP clock */
	CLK_EnableModuleClock(UART0_MODULE);
	
	/* Select IP clock source */
	CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

	/* Set GPA multi-function pins for UART0 RXD(GPA.6) and TXD(GPA.7) */
	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
	SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA6MFP_UART0_RXD | SYS_GPA_MFPL_PA7MFP_UART0_TXD);
	
	/* Init UART to 115200-8n1 for print message */
	UART_Open(UART0, 115200);

#if 1
	/* Enable IP clock */
	CLK_EnableModuleClock(UART2_MODULE);
	
	/* Select IP clock source */
	CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HXT, CLK_CLKDIV4_UART2(1));

	/* Set GPC multi-function pins for UART0 RXD(GPC.4) and TXD(GPC.5) */
	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk);
	SYS->GPC_MFPL |= SYS_GPC_MFPL_PC4MFP_UART2_RXD | SYS_GPC_MFPL_PC5MFP_UART2_TXD;
	
	/* Init UART to 115200-8n1 for print message */
	UART_Open(UART2, 115200);
#endif
	
} /* End of SYS_Init() */

void NMI_Handler() { while (1); }
void HardFault_Handler() { while (1); }
void MemManage_Handler() { while (1); }
void BusFault_Handler() { while (1); }
void UsageFault_Handler() { while (1); }
void SVC_Handler() { while (1); }
void DebugMon_Handler() { while (1); }
void PendSV_Handler() { while (1); }

void SysTick_Handler() {
}

//	BOD_IRQHandler
//	IRC_IRQHandler
//	PWRWU_IRQHandler
//	RAMPE_IRQHandler
//	CKFAIL_IRQHandler
//	RTC_IRQHandler
//	TAMPER_IRQHandler
void WDT_IRQHandler() { while(1); }
void WWDT_IRQHandler() { while(1); }
//	EINT0_IRQHandler
//	EINT1_IRQHandler
//	EINT2_IRQHandler
//	EINT3_IRQHandler
//	EINT4_IRQHandler
//	EINT5_IRQHandler
//	GPA_IRQHandler
//	GPB_IRQHandler
//	GPC_IRQHandler
//	GPD_IRQHandler
//	GPE_IRQHandler
//	GPF_IRQHandler
//	QSPI0_IRQHandler
//	SPI0_IRQHandler
//	BRAKE0_IRQHandler
//	EPWM0P0_IRQHandler
//	EPWM0P1_IRQHandler
//	EPWM0P2_IRQHandler
//	BRAKE1_IRQHandler
//	EPWM1P0_IRQHandler
//	EPWM1P1_IRQHandler
//	EPWM1P2_IRQHandler
void TMR0_IRQHandler() { while(1); }
void TMR1_IRQHandler() { while(1); }
void TMR2_IRQHandler() { while(1); }
void TMR3_IRQHandler() { while(1); }
void UART0_IRQHandler() { while(1); }
void UART1_IRQHandler() { while(1); }
//	I2C0_IRQHandler
//	I2C1_IRQHandler
//	PDMA_IRQHandler
//	DAC_IRQHandler
//	EADC00_IRQHandler
//	EADC01_IRQHandler
//	ACMP01_IRQHandler
//	EADC02_IRQHandler
//	EADC03_IRQHandler
void UART2_IRQHandler() { while(1); }
void UART3_IRQHandler() { while(1); }
//	QSPI1_IRQHandler
//	SPI1_IRQHandler
//	SPI2_IRQHandler
//	USBD_IRQHandler
//	OHCI_IRQHandler
//	USBOTG_IRQHandler
//	CAN0_IRQHandler
//	CAN1_IRQHandler
//	SC0_IRQHandler
//	SC1_IRQHandler
//	SC2_IRQHandler
//	SPI3_IRQHandler
//	SDH0_IRQHandler
//	USBD20_IRQHandler
//	EMAC_TX_IRQHandler
//	EMAC_RX_IRQHandler
//	I2S0_IRQHandler
//	OPA0_IRQHandler
//	CRYPTO_IRQHandler
//	GPG_IRQHandler
//	EINT6_IRQHandler
//	UART4_IRQHandler
//	UART5_IRQHandler
//	USCI0_IRQHandler
//	USCI1_IRQHandler
//	BPWM0_IRQHandler
//	BPWM1_IRQHandler
//	SPIM_IRQHandler
//	I2C2_IRQHandler
//	QEI0_IRQHandler
//	QEI1_IRQHandler
//	ECAP0_IRQHandler
//	ECAP1_IRQHandler
//	GPH_IRQHandler
//	EINT7_IRQHandler
//	SDH1_IRQHandler
//	EHCI_IRQHandler
//	USBOTG20_IRQHandler
//	TRNG_IRQHandler
//	UART6_IRQHandler
//	UART7_IRQHandler
//	EADC10_IRQHandler
//	EADC11_IRQHandler
//	EADC12_IRQHandler
//	EADC13_IRQHandler
//	CAN2_IRQHandler

int _write(int fd, char* ptr, int len)
{
    UART_Write(UART2, ptr, len);
}
