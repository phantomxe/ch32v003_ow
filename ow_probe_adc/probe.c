#define SYSTEM_CORE_CLOCK 48000000
#define APB_CLOCK SYSTEM_CORE_CLOCK 
#define SYSTICK_USE_HCLK 

#include "ch32v003fun.h" 
#include "ch32v003_GPIO_branchless.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  

#define ONEWIRE_PIN_RELEASE()	 ({GPIOC->CFGLR &= ~(0xf<<(4*1)); GPIOC->CFGLR |= (GPIO_CNF_IN_FLOATING)<<(4*1);})
#define ONEWIRE_PIN_DRIVE_LOW()  ({GPIOC->CFGLR &= ~(0xf<<(4*1)); GPIOC->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_OD)<<(4*1); GPIOC->BSHR = (1<<(16+1));}) //PC1 Open Drain Set Low
#define ONEWIRE_PIN_READ()		 ((GPIOC->INDR >> 1) & 1)

#define ADC_NUMCHLS 1 
volatile uint16_t adc_buffer[ADC_NUMCHLS]; 

volatile uint8_t rom[8] = {0x28, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x11, 0x00}; 
volatile uint8_t scratchpad[10] = {0x89, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0x0E, 0x00};  

typedef enum  {
  WAIT_FOR_RESET,
  ROM_CMD,
  FUNCTION_CMD,
} state_t;

volatile state_t current_state = WAIT_FOR_RESET;  

volatile uint8_t buffer, conv_ct = 0;
volatile uint32_t falling = 0, entering_time = 0;    
volatile uint8_t write_mode = 0, write_waiting_bytes = 0, write_current_byte= 0, write_waiting_size = 0, write_waiting_bit_size = 8;

void adc_init( void );
uint8_t read_bit (uint8_t);
uint8_t read_byte (void); 
void EXTI7_0_IRQHandler( void ) __attribute__((interrupt)); // __attribute__((section(".srodata")));
 
void EXTI7_0_IRQHandler( void ) 
{         
  //asm volatile( "" : : : "memory" );
 
  if(write_mode == 1) {  
    GPIO_digitalWrite(GPIO_port_C, 4, high); 

    if(write_waiting_bit_size == 0) {
      write_waiting_bit_size = 8; 
      write_current_byte = scratchpad[write_waiting_bytes]; 
    }

    GPIO_digitalWrite(GPIO_port_C, 2, high);  //for debugging

    if(write_current_byte & 0x01) { 
      Delay_Us(40);
    } else {  
      ONEWIRE_PIN_DRIVE_LOW();
      Delay_Us(15);
      ONEWIRE_PIN_RELEASE(); 
    } 

    GPIO_digitalWrite(GPIO_port_C, 2, low); //for debugging 

    write_current_byte >>= 1;
    write_waiting_bit_size--;   

    if(write_waiting_bit_size == 0) {
      write_waiting_bytes++; 
    }

    entering_time = SysTick->CNT;

    if(write_waiting_bit_size == 0 && write_waiting_bytes >= 9) {
      GPIO_digitalWrite(GPIO_port_C, 4, low);  // For debugging
      write_waiting_bytes = 0; 
      write_current_byte = 0;
      write_waiting_bit_size = 8;
      write_mode = 0;
      EXTI->FTENR = 1<<(1); 
      EXTI->INTFR = 1<<1; 
      return;
    }

    EXTI->INTFR = 1<<1;
    return; 
  } else {
    GPIO_digitalWrite(GPIO_port_C, 4, low);  // For debugging
  }


  if(current_state == WAIT_FOR_RESET) {
    if(ONEWIRE_PIN_READ()) { 
      int32_t pulse_width = (int32_t)(SysTick->CNT - falling) / 48;
      if(pulse_width < -1000 || pulse_width > 1000) { 
        EXTI->INTFR = 1<<1;
        return;
      }
      
      if(pulse_width > 480) {
        // this is a reset pulse
        // send presence pulse 
        Delay_Us(30);
        ONEWIRE_PIN_DRIVE_LOW();
        Delay_Us(65);
        ONEWIRE_PIN_RELEASE();
        Delay_Us(370);   

        current_state = ROM_CMD;  
      }   
    } else {
      falling = SysTick->CNT; 
    }
  } else if(current_state == ROM_CMD) {   
    buffer = read_byte();    
    Delay_Us(2);

    if(buffer == 0xcc) { 
      buffer = read_byte();   

      if(buffer == 0x44) {  

        scratchpad[0] = (uint8_t)(adc_buffer[0] & 0xFF); // Not working sometimes
        scratchpad[1] = (uint8_t)((adc_buffer[0] >> 8) & 0xFF);  

        scratchpad[2] = 0x31; // for debugging

        current_state = WAIT_FOR_RESET; 
      } else if(buffer == 0xbe) { 
        write_mode = 1;
        write_current_byte = scratchpad[0];
        entering_time = SysTick->CNT;
        EXTI->FTENR = 0;
        current_state = WAIT_FOR_RESET; 
      } 
    } else {
      current_state = WAIT_FOR_RESET;
    } 
  }  

  EXTI->INTFR = 1<<1; 
} 

uint8_t read_bit (uint8_t id) {  
    uint8_t bit;    
                 
    Delay_Us(22);  
    bit = ONEWIRE_PIN_READ();  
    Delay_Us(45);  
   
    return bit;
}

uint8_t read_byte (void) {   
	uint8_t result=0; 
	if (read_bit(0))
		result |= 0x80;				 
	result >>= 1; 				 
	if (read_bit(1))
		result |= 0x80; 
	result >>= 1;
	if (read_bit(2))
		result |= 0x80; 
	result >>= 1;
	if (read_bit(3))
		result |= 0x80; 
	result >>= 1;
	if (read_bit(4))
		result |= 0x80; 
	result >>= 1;
	if (read_bit(5))
		result |= 0x80; 
	result >>= 1;
	if (read_bit(6))
		result |= 0x80; 
	result >>= 1;
	if (read_bit(7))
		result |= 0x80;
	return result; 
} 

void adc_init( void )
{ 
  // ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
	RCC->CFGR0 &= ~(0x1F<<11);
	// PD6 is analog input chl 6
	GPIOD->CFGLR &= ~(0xf<<(4*6));	// CNF = 00: Analog, MODE = 00: Input 
	
	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1; 
	
	// Set up four conversions on chl 7, 4, 3, 2
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = GPIO_Ain6_D6;
	
	//ADC1->SAMPTR2 &= ~(ADC_SMP0<<(3*6));
	ADC1->SAMPTR2 |= 7<<(3*6);	// 0:7 => 3/9/15/30/43/57/73/241 cycles

	// turn on ADC
	ADC1->CTLR2 |= ADC_ADON;
	
	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	
	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);
	
	// Turn on DMA
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;
	
	//DMA1_Channel1 is for ADC
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
	DMA1_Channel1->MADDR = (uint32_t)adc_buffer;
	DMA1_Channel1->CNTR  = ADC_NUMCHLS;
	DMA1_Channel1->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_VeryHigh |
		DMA_MemoryDataSize_HalfWord |
		DMA_PeripheralDataSize_HalfWord |
		DMA_MemoryInc_Enable |
		DMA_Mode_Circular |
		DMA_DIR_PeripheralSRC;
	
	// Turn on DMA channel 1
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN;
	
	// enable scanning
	ADC1->CTLR1 |= ADC_SCAN;
	
	// Enable continuous conversion and DMA
	ADC1->CTLR2 |= ADC_CONT | ADC_DMA | ADC_EXTSEL;
	
	// start conversion
	ADC1->CTLR2 |= ADC_SWSTART;

    printf("ADC init done\r\n");
} 


int main()
{ 
    SystemInit48HSI();
    SetupDebugPrintf();

    SETUP_SYSTICK_HCLK

    Delay_Ms( 100 );   

    printf("ow-slave adc demo\r\n"); 

    RCC->APB2PCENR = RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1; 

    adc_init();

    AFIO->EXTICR = 2<<(1*2); // PC1: PortC = 2, << Pin1 = 1 * 2 
    EXTI->INTENR = 1<<(1); // EXT1 Enable
    EXTI->RTENR  = 1<<(1); // EXT1 Rising edge trigger
    EXTI->FTENR  = 1<<(1); // EXT1 Falling edge trigger

    ONEWIRE_PIN_RELEASE();

    GPIO_portEnable(GPIO_port_C);
    
    // For debugging
    GPIO_pinMode(GPIO_port_C, 2, GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
    GPIO_pinMode(GPIO_port_C, 4, GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
    GPIO_digitalWrite(GPIO_port_C, 2, low); 
    GPIO_digitalWrite(GPIO_port_C, 4, low);
 
    asm volatile("addi t1, x0, 0\ncsrrw x0, 0x804, t1\n" : : : "t1");

    NVIC_EnableIRQ( EXTI7_0_IRQn )  

    while(1) {     
        //onewire write mode timeout
        if(write_mode == 1 && (int32_t)(SysTick->CNT - entering_time) > 480000 ) {   
          write_waiting_bytes = 0;
          write_current_byte = 0;
          write_waiting_bit_size = 8;
          write_mode = 0; 
          EXTI->FTENR = 1<<(1);  
          EXTI->INTFR = 1<<1;
          return;
        }    
    }
}
