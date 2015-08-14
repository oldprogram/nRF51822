#ifndef NRF_GPIO_H__
#define NRF_GPIO_H__

#include "nrf51.h"
#include "nrf51_bitfields.h"

/**
 * @defgroup nrf_gpio GPIO abstraction
 * @{
 * @ingroup nrf_drivers
 * @brief GPIO pin abstraction and port abstraction for reading and writing byte-wise to GPIO ports.
 *
 * Here, the GPIO ports are defined as follows:
 * - Port 0 -> pin 0-7
 * - Port 1 -> pin 8-15
 * - Port 2 -> pin 16-23
 * - Port 3 -> pin 24-31
 */

/**
 * @enum nrf_gpio_port_dir_t
 * @brief Enumerator used for setting the direction of a GPIO port.
 */
typedef enum
{
    NRF_GPIO_PORT_DIR_OUTPUT,       ///<  Output
    NRF_GPIO_PORT_DIR_INPUT         ///<  Input
} nrf_gpio_port_dir_t;

/**
 * @enum nrf_gpio_pin_dir_t
 * Pin direction definitions.
 */
typedef enum
{
    NRF_GPIO_PIN_DIR_INPUT,   ///< Input
    NRF_GPIO_PIN_DIR_OUTPUT   ///< Output
} nrf_gpio_pin_dir_t;

/**
 * @enum nrf_gpio_port_select_t
 * @brief Enumerator used for selecting between port 0 - 3.
 */
typedef enum
{
    NRF_GPIO_PORT_SELECT_PORT0 = 0,           ///<  Port 0 (GPIO pin 0-7)
    NRF_GPIO_PORT_SELECT_PORT1,               ///<  Port 1 (GPIO pin 8-15)
    NRF_GPIO_PORT_SELECT_PORT2,               ///<  Port 2 (GPIO pin 16-23)
    NRF_GPIO_PORT_SELECT_PORT3,               ///<  Port 3 (GPIO pin 24-31)
} nrf_gpio_port_select_t;

/**
 * @enum nrf_gpio_pin_pull_t
 * @brief Enumerator used for selecting the pin to be pulled down or up at the time of pin configuration
 */
typedef enum
{
    NRF_GPIO_PIN_NOPULL   = GPIO_PIN_CNF_PULL_Disabled,                 ///<  Pin pullup resistor disabled
    NRF_GPIO_PIN_PULLDOWN = GPIO_PIN_CNF_PULL_Pulldown,                 ///<  Pin pulldown resistor enabled
    NRF_GPIO_PIN_PULLUP   = GPIO_PIN_CNF_PULL_Pullup,                   ///<  Pin pullup resistor enabled
} nrf_gpio_pin_pull_t;

/**
 * @brief Configure GPIO pin range as outputs with normal drive strength.
 *        This function can be used to configure pin range as simple output with gate driving GPIO_PIN_CNF_DRIVE_S0S1 (normal cases).
 *
 * @param pin_range_start specifies the start number (inclusive) in the range of pin numbers to be configured (allowed values 0-30)
 *
 * @param pin_range_end specifies the end number (inclusive) in the range of pin numbers to be configured (allowed values 0-30)
 *
 * @note For configuring only one pin as output use @ref nrf_gpio_cfg_output
 *       Sense capability on the pin is disabled, and input is disconnected from the buffer as the pins are configured as output.
 */
static __INLINE void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        NRF_GPIO->PIN_CNF[pin_range_start] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                        | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
    }
}

/**
 * @brief Configure GPIO pin range as inputs with given initial value set, hiding inner details.
 *        This function can be used to configure pin range as simple input.
 *
 * @param pin_range_start specifies the start number (inclusive) in the range of pin numbers to be configured (allowed values 0-30)
 *
 * @param pin_range_end specifies the end number (inclusive) in the range of pin numbers to be configured (allowed values 0-30)
 *
 * @param pull_config State of the pin range pull resistor (no pull, pulled down or pulled high)
 *
 * @note  For configuring only one pin as input use @ref nrf_gpio_cfg_input
 *        Sense capability on the pin is disabled, and input is connected to buffer so that the GPIO->IN register is readable
 */
static __INLINE void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        NRF_GPIO->PIN_CNF[pin_range_start] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                        | (pull_config << GPIO_PIN_CNF_PULL_Pos)
                                        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                        | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    }
}

/**
 * @brief Configure given GPIO pin number as output with given initial value set, hiding inner details.
 *        This function can be used to configure pin range as simple input with gate driving GPIO_PIN_CNF_DRIVE_S0S1 (normal cases).
 *
 * @param pin_number specifies the pin number of gpio pin numbers to be configured (allowed values 0-30)
 *
 * @note  Sense capability on the pin is disabled, and input is disconnected from the buffer as the pins are configured as output.
 */
static __INLINE void nrf_gpio_cfg_output(uint32_t pin_number)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
}

/**
 * @brief Configure given GPIO pin number as input with given initial value set, hiding inner details.
 *        This function can be used to configure pin range as simple input with gate driving GPIO_PIN_CNF_DRIVE_S0S1 (normal cases).
 *
 * @param pin_number specifies the pin number of gpio pin numbers to be configured (allowed values 0-30)
 *
 * @param pull_config State of the pin range pull resistor (no pull, pulled down or pulled high)
 *
 * @note  Sense capability on the pin is disabled, and input is connected to buffer so that the GPIO->IN register is readable
 */
static __INLINE void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[pin_number] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                        | (pull_config << GPIO_PIN_CNF_PULL_Pos)
                                        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                        | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}

/**
 * @brief Set the direction for a GPIO pin.
 *
 * @param pin_number specifies the pin number [0:31] for which to
 * set the direction.
 *
 * @param direction specifies the direction
 */
static __INLINE void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction)
{
    if(direction == NRF_GPIO_PIN_DIR_INPUT)
    {
        NRF_GPIO->PIN_CNF[pin_number] =
          (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
        | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    }
    else
    {
        NRF_GPIO -> DIRSET = (1UL << pin_number);
    }
}

/**
 * @brief Set GPIO pin.
 *
 * Note that the pin must be configured as an output for this
 * function to have any effect.
 *
 * @param pin_number specifies the pin number [0:31] to
 * set.
 */
static __INLINE void nrf_gpio_pin_set(uint32_t pin_number)
{
    NRF_GPIO->OUTSET = (1UL << pin_number);
}

/**
 * @brief Clear GPIO pin.
 *
 * Note that the pin must be configured as an output for this
 * function to have any effect.
 *
 * @param pin_number specifies the pin number [0:31] to
 * clear.
 */
static __INLINE void nrf_gpio_pin_clear(uint32_t pin_number)
{
    NRF_GPIO->OUTCLR = (1UL << pin_number);
}

/**
 * @brief Toggle GPIO pin.
 *
 * Note that the pin must be configured as an output for this
 * function to have any effect.
 *
 * @param pin_number specifies the pin number [0:31] to
 * toggle.
 */
static __INLINE void nrf_gpio_pin_toggle(uint32_t pin_number)
{
    NRF_GPIO->OUT ^= (1UL << pin_number);
}

/**
 * @brief Write value to GPIO pin.
 *
 * Note that the pin must be configured as an output for this
 * function to have any effect.
 *
 * @param pin_number specifies the pin number [0:31] to
 * write.
 *
 * @param value specifies the value to be written to the pin.
 * @arg 0 clears the pin
 * @arg >=1 sets the pin.
 */
static __INLINE void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value)
{
    if (value == 0)
    {
        nrf_gpio_pin_clear(pin_number);
    }
    else
    {
        nrf_gpio_pin_set(pin_number);
    }
}

/**
 * @brief Read the input level of a GPIO pin.
 *
 * Note that the pin must have input connected for the value
 * returned from this function to be valid.
 *
 * @param pin_number specifies the pin number [0:31] to
 * read.
 *
 * @return
 * @retval 0 if the pin input level is low.
 * @retval 1 if the pin input level is high.
 * @retval > 1 should never occur.
 */
static __INLINE uint32_t nrf_gpio_pin_read(uint32_t pin_number)
{
    return  ((NRF_GPIO->IN >> pin_number) & 1UL);
}

/**
 * @brief Generic function for writing a single byte of a 32 bit word at a given
 * address.
 *
 * This function should not be called from outside the nrf_gpio
 * abstraction layer.
 *
 * @param word_address is the address of the word to be written.
 *
 * @param byte_no is the the word byte number (0-3) to be written.
 *
 * @param value is the value to be written to byte "byte_no" of word
 * at address "word_address"
 */
static __INLINE void nrf_gpio_word_byte_write(volatile uint32_t * word_address, uint8_t byte_no, uint8_t value)
{
    *((uint8_t*)(word_address) + byte_no) = value;
}

/**
 * @brief Generic function for reading a single byte of a 32 bit word at a given
 * address.
 *
 * This function should not be called from outside the nrf_gpio
 * abstraction layer.
 *
 * @param word_address is the address of the word to be read.
 *
 * @param byte_no is the the byte number (0-3) of the word to be read.
 *
 * @return byte "byte_no" of word at address "word_address".
 */
static __INLINE uint8_t nrf_gpio_word_byte_read(const volatile uint32_t* word_address, uint8_t byte_no)
{
    return (*((uint8_t*)(word_address) + byte_no));
}

/**
 * @brief Set the direction of a port.
 *
 * @param port is the port for which to set the direction.
 *
 * @param dir direction to be set for this port.
 */
static __INLINE void nrf_gpio_port_dir_set(nrf_gpio_port_select_t port, nrf_gpio_port_dir_t dir)
{
    if (dir == NRF_GPIO_PORT_DIR_OUTPUT)
    {
        nrf_gpio_word_byte_write(&NRF_GPIO->DIRSET, port, 0xFF);
    }
    else
    {
        nrf_gpio_range_cfg_input(port*8, (port+1)*8-1, NRF_GPIO_PIN_NOPULL);
    }
}

/**
 * @brief Read GPIO port.
 *
 * @param port is the port to read.
 *
 * @return the input value on this port.
 */
static __INLINE uint8_t nrf_gpio_port_read(nrf_gpio_port_select_t port)
{
    return nrf_gpio_word_byte_read(&NRF_GPIO->IN, port);
}

/**
 * @brief Write GPIO port.
 *
 * @param port is the port to write.
 *
 * @param value is the value to write to this port.
 *
 * @sa nrf_gpio_port_dir_set()
 */
static __INLINE void nrf_gpio_port_write(nrf_gpio_port_select_t port, uint8_t value)
{
    nrf_gpio_word_byte_write(&NRF_GPIO->OUT, port, value);
}

/**
 * @brief Set individual pins on GPIO port.
 *
 * @param port is the port for which to set the pins.
 *
 * @param set_mask is a mask specifying which pins to set. A bit
 * set to 1 indicates that the corresponding port pin shall be
 * set.
 *
 * @sa nrf_gpio_port_dir_set()
 */
static __INLINE void nrf_gpio_port_set(nrf_gpio_port_select_t port, uint8_t set_mask)
{
    nrf_gpio_word_byte_write(&NRF_GPIO->OUTSET, port, set_mask);
}

/**
 * @brief Clear individual pins on GPIO port.
 *
 * @param port is the port for which to clear the pins.
 *
 * @param clr_mask is a mask specifying which pins to clear. A bit
 * set to 1 indicates that the corresponding port pin shall be
 * cleared.
 *
 * @sa nrf_gpio_port_dir_set()
 */
static __INLINE void nrf_gpio_port_clear(nrf_gpio_port_select_t port, uint8_t clr_mask)
{
    nrf_gpio_word_byte_write(&NRF_GPIO->OUTCLR, port, clr_mask);
}

/** @} */

#endif
