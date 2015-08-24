#include "sensor.h"
#include "nrf_gpio.h"
#include "boards.h"


#if 1
/** @brief Function for handling the GPIOTE interrupt which is triggered on pin 0 change.
 */
void GPIOTE_IRQHandler(void)
{
    // Event causing the interrupt must be cleared.
    if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && 
        (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;

    }
		nrf_gpio_pin_toggle(IND_LED);// any interrupt toggle LED
		
    uint32_t state_l, state_r, state_up, state_down; //
	//disable interrupt
	//clear interrupt 
	//read sensor gpio state,format  the 4 together
	state_l = nrf_gpio_pin_read(SEN_LEFT);
	
	state_r = nrf_gpio_pin_read(SEN_RIGHT);
	state_up = nrf_gpio_pin_read(SEN_UP);
	state_down   = nrf_gpio_pin_read(SEN_DOWN);
	#if 0
	//callback function

	//call BT stack if connected
		
	if(state_l == 1)
		//display by call lcd function if not connected
	  ;
	if(state_r == 1)
		//display by call lcd function if not connected
	  ;
	if(state_up == 1)
		//display by call lcd function if not connected
	  ;
	if(state_down == 1)
		//display by call lcd function if not connected
	  ;
	//enable interrupt
	//return 0;
	#endif
}
#endif
void sensor_init(void)
{
	nrf_gpio_cfg_output(IND_LED);
	nrf_gpio_pin_clear(IND_LED);// any interrupt toggle LED
	//config 5 io as input
	nrf_gpio_cfg_input(SEN_LEFT,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(SEN_RIGHT,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(SEN_UP,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(SEN_DOWN,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(SEN_INT,NRF_GPIO_PIN_PULLUP);
  
	nrf_gpio_cfg_output(SEN_POWER_EN);
	nrf_gpio_pin_set(SEN_POWER_EN);//
	//4 pull down 1 pullup
	//register callback for int pin
	NVIC_DisableIRQ(GPIOTE_IRQn);
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	
	//may create 4 interrupts like following, toggle measn high->low and low-high
	//NVIC_EnableIRQ(GPIOTE_IRQn);
  NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
                           | (SEN_INT << GPIOTE_CONFIG_PSEL_Pos)  
                           | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
  NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
	 __NOP();
   __NOP();
   __NOP();
	 /* Clear the event that appears in some cases */
   NRF_GPIOTE->EVENTS_IN[0] = 0; 
	 NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos;
	 NVIC_EnableIRQ(GPIOTE_IRQn);
}
