/*
 * PI4IOE5V6534Q.c
 *
 *  Created on: 15 Jul. 2023
 *      Author: Ethan
 *  Datasheet: https://www.diodes.com/assets/Datasheets/PI4IOE5V6534Q.pdf
 */

/* TODO: Change from HAL_MAX_DELAY
 * Add bulk pin direction change function (for all ports) - Maybe
 * Add Struct defining a pin that stores all config
 * Add function that takes above struct and writes it's config to the peripheral
*/

#include "PI4IOE5V6534Q.h"

HAL_StatusTypeDef PI4IO_read_reg(PI4IOPtr const PI4IO_Instance, uint8_t reg_num, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Master_Transmit(PI4IO_Instance->hi2cx, PI4IO_Instance->i2c_addr << 1 | PI4IO_CMD_READ_REG, &reg_num, 1, HAL_MAX_DELAY);

	if(ret != HAL_OK) {
		return ret;
	} else {
		ret = HAL_I2C_Master_Receive(PI4IO_Instance->hi2cx, PI4IO_Instance->i2c_addr << 1 | PI4IO_CMD_READ_REG, pvalue, 1, HAL_MAX_DELAY);

		return ret;
	}

}

HAL_StatusTypeDef PI4IO_write_reg(PI4IOPtr const PI4IO_Instance, uint8_t reg_num, uint8_t value) {
	HAL_StatusTypeDef ret;

	uint8_t buf[2] = {reg_num, value};

	ret = HAL_I2C_Master_Transmit(PI4IO_Instance->hi2cx, PI4IO_Instance->i2c_addr << 1 | PI4IO_CMD_WRITE_REG, buf, 2, HAL_MAX_DELAY);

	return ret;
}

/* Software reset
 * Resets all registers to default states
 */

HAL_StatusTypeDef PI4IO_soft_reset(PI4IOPtr const PI4IO_Instance) {
	HAL_StatusTypeDef ret;

	uint8_t reset_command = PI4IO_CMD_RESET;

	ret = HAL_I2C_Master_Transmit(PI4IO_Instance->hi2cx, 0x00, &reset_command, 1, HAL_MAX_DELAY);

	return ret;
}

/*
 * Pin Direction
 * 0 - Output
 * 1 - Input
 *
 * Default - 1
 */

HAL_StatusTypeDef PI4IO_get_port_pins_direction(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {

	HAL_StatusTypeDef ret;

	uint8_t port_reg = PI4IO_REG_CONFIG_PORT_0 + port;

	ret = PI4IO_read_reg(PI4IO_Instance, port_reg, pvalue);

	return ret;

}

/*
 * @brief  bulk updates the given IOEXP port to set the pins as input and output
 * @param  port The pins you are attempting to modify belong to this port [Range: 0-4 inclusive]
 * @param  pins 8 bit value where each bit represents a pin, LSB is pin 0, MSB is pin 8
 * 		        a value of 0 for a pin represents Output, 1 represents input
 * @retVal Hal Status of operation
 */

HAL_StatusTypeDef PI4IO_set_port_pins_direction(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	uint8_t port_reg = PI4IO_REG_CONFIG_PORT_0 + port;

	ret = PI4IO_write_reg(PI4IO_Instance, port_reg, pins);

	return ret;
}

/*
 * @brief  Updates the given IOEXP port to set the given pin as input or output
 * @param  port_pin 8 bit value, upper most 4 bits is the port [Range: 0-4 inclusive]
 *         lower 4 bits is the pin, range for ports 0,1,2,3 is 0-7 and 0-1 for port 4 (inclusive)
 * @param  direction A value of 0 for a pin represents Output, 1 represents input
 * @retVal Hal Status of operation
 */
HAL_StatusTypeDef PI4IO_set_single_pin_direction(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, PinDirection direction) {
	HAL_StatusTypeDef ret;

	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);

	uint8_t current_port_config = 0x00;

	// Get current port's pin values
	PI4IO_get_port_pins_direction(PI4IO_Instance, port, &current_port_config);

	// Check if the pin is already in the desired direction
	if(bitcheck(current_port_config, pin) == direction) {
		return HAL_OK;
	} else {
		// Modify existing config to set the new config
		bitflip(current_port_config, pin);

		//ret = PI4IO_write_reg(PI4IO_Instance, port_reg, current_port_config);
		ret = PI4IO_set_port_pins_direction(PI4IO_Instance, port, current_port_config);
		return ret;

	}


}

/* Input polarity
 * 0 - Normal (Input pins will register 1 for not-closed and 0 for closed)
 * 1 - Inverted (Input pins will register 0 for not-closed and 1 for closed)
 *
 * Default - 0
 */


// Input pins will register 1 for not-closed and 0 for closed
// If port = 0xff do all ports and all pins
// if pins = 0xff do all pins for that port
// Note: Any 0's in the pins parameter will be written to the register, i.e setting those pins to inverted instead of normal
// It won't keep the previous polarity of those pins
void PI4IO_set_input_port_polarity_normal(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {

	if(port > 4) {
		// Set polarity to normal for all ports
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_0, 0x00);
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_1, 0x00);
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_2, 0x00);
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_3, 0x00);
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_4, 0x00);
	} else {
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_0 + port, ~pins); // Not pins as we want to write 0 to the reg for all pins labelled with 1 in the pins variable
	}

}


/* @brief  Function that writes the current port's pin polarity to variable pointed to by pvalue
 *
 */
HAL_StatusTypeDef PI4IO_get_input_port_polarity(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_0 + port, pvalue);

	return ret;

}

// Input pins will register 0 for not-closed and 1 for closed
// If port = 0xff do all ports and all pins
// if pins = 0xff do all pins for that port
// Note: Any 0's in the pins parameter will be written to the register, i.e setting those pins to normal instead of inverted
// It won't keep the previous polarity of those pins
void PI4IO_set_input_port_polarity_inverted(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {

	if(port > 4) {
		// Set polarity to inverted for all ports
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_0, 0xff);
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_1, 0xff);
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_2, 0xff);
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_3, 0xff);
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_4, 0xff);
	} else {
		PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_POLARITY_INVERSION_PORT_0 + port, pins);
	}
}

/* @brief  Function that sets the polarity of an individual pin
 *
  *@param  port_pin 8 bit value, upper most 4 bits is the port [Range: 0-4 inclusive]
 *         lower 4 bits is the pin, range for ports 0,1,2,3 is 0-7 and 0-1 for port 4 (inclusive)
 */
HAL_StatusTypeDef PI4IO_set_input_pin_polarity(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, InputPolarity ip) {
	HAL_StatusTypeDef ret;

	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t port_reg = PI4IO_REG_POLARITY_INVERSION_PORT_0 + port;
	uint8_t current_port_config = 0x00;

	PI4IO_get_input_port_polarity(PI4IO_Instance, port, &current_port_config);

	if(ip == INPUT_POLARITY_INVERTED) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	ret = PI4IO_write_reg(PI4IO_Instance, port_reg, current_port_config);

	return ret;


}

/* Output drive level
 * 0 - 25%  Current
 * 1 - 50%  current
 * 2 - 75%  current
 * 3 - 100% current
 *
 * Default - 3
 */

HAL_StatusTypeDef PI4IO_get_output_pin_drive_level(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t port_reg = (PI4IO_REG_OUTPUT_DRIVE_LEVEL_0A + (port * 2) + (pin / 4)); // Determine which register corresponds to the port/pin combination
	ret = PI4IO_read_reg(PI4IO_Instance, port_reg, pvalue);

	if(pin % 2 == 0) {
		// Pin number is even
		if(pin == 0 || pin == 4) {
			// Pin is 0 or 4 (Stored in bits 0-1)
			*pvalue &= 0x03; // Mask 2 LSB
		} else {
			// Pin is 2 or 6 (Stored in bits 4-5)
			*pvalue >>= 4;
			*pvalue &= 0x03; // Mask 2 LSB
		}
	} else {
		// Pin number is odd
		if(pin == 1 || pin == 5) {
			// pin is 1 or 5 (Stored in bits 2-3)
			*pvalue >>= 2;
			*pvalue &= 0x03; // Mask 2 LSB
		} else {
			// pin is 3 or 7 (Stored in bits 6-7)
			*pvalue >>= 6;
		}
	}

	return ret;

}

HAL_StatusTypeDef PI4IO_set_output_pin_drive_level(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, OutputDriveLevel odl) {
	HAL_StatusTypeDef ret;

	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t port_reg = (PI4IO_REG_OUTPUT_DRIVE_LEVEL_0A + (port * 2) + (pin / 4)); // Determine which register corresponds to the port/pin combination
	uint8_t current_port_config = 0x00;

	PI4IO_read_reg(PI4IO_Instance, port_reg, &current_port_config);

	uint8_t shiftamt = pin * 2 - ((pin > 3) * 8); // 2 bits per pin, 2 registers per port, we subtract 6 from shiftamt if the register is the second one for the port

	bitclear(current_port_config, shiftamt);
	bitclear(current_port_config, shiftamt+1);

	current_port_config |= (odl << shiftamt);


	ret = PI4IO_write_reg(PI4IO_Instance, port_reg, current_port_config);

	return ret;
}


void PI4IO_set_output_port_drive_level(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins[2] ) {
	uint8_t port_reg = PI4IO_REG_OUTPUT_DRIVE_LEVEL_0A + (port * 2);

	PI4IO_write_reg(PI4IO_Instance, port_reg, pins[0]);
	PI4IO_write_reg(PI4IO_Instance, port_reg, pins[1]);

	return;
}

/* Input latch state
 * 0 - Unlatched
 * 1 - Latched
 *
 * Default - 0
 */

HAL_StatusTypeDef PI4IO_get_port_input_latch_state(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_INPUT_LATCH_PORT_0 + port, pvalue);

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_input_latch_state(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_INPUT_LATCH_PORT_0 + port, pins);

	return ret;
}


void PI4IO_set_pin_input_latch_state(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, InputLatchState ls) {

	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_input_latch_state(PI4IO_Instance, port, &current_port_config);

	if(ls == INPUT_LATCHED) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	PI4IO_set_port_input_latch_state(PI4IO_Instance, port, current_port_config);

	return;
}

/* Pull-up/Pull-down resistor enable/disable
 * 0 - Disabled
 * 1 - Enabled
 *
 * Default - 0
 */

HAL_StatusTypeDef PI4IO_get_port_resistor_enable(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_PULLUP_PULLDOWN_ENABLE_PORT_0 + port, pvalue);

	return ret;

}

HAL_StatusTypeDef PI4IO_set_port_resistor_enable(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_PULLUP_PULLDOWN_ENABLE_PORT_0 + port, pins);

	return ret;
}

void PI4IO_set_pin_resistor_enable(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, ResistorState rs) {
	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_resistor_enable(PI4IO_Instance, port, &current_port_config);

	if(rs == RESISTOR_ENABLE) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	PI4IO_set_port_resistor_enable(PI4IO_Instance, port, current_port_config);

	return;
}

/* Pull-up/Pull-down resistor selection
 * 0 - 100k pull-up resistor (Min 50k max 150k)
 * 1 - 100k pull-down resistor (Min 50k max 150k)
 *
 * Default - 1
 */

HAL_StatusTypeDef PI4IO_get_port_resistor_selection(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_PULLUP_PULLDOWN_SELECT_PORT_0 + port, pvalue);

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_resistor_selection(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_PULLUP_PULLDOWN_SELECT_PORT_0 + port, pins);

	return ret;
}

void setPI4IO_set_pin_resistor_selection(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, ResistorType rt) {
	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_resistor_selection(PI4IO_Instance, port, &current_port_config);

	if(rt == PULLUP_100k) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	PI4IO_set_port_resistor_selection(PI4IO_Instance, port, current_port_config);

	return;

}

/* Interrupt masking
 * 0 - Enabled
 * 1 - Disabled
 *
 * Default - 1
 */

HAL_StatusTypeDef PI4IO_get_port_interrupt_mask(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_INTERRUPT_MASK_PORT_0 + port, pvalue);

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_interrupt_mask(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_INTERRUPT_MASK_PORT_0 + port, pins);

	return ret;
}

void PI4IO_set_pin_interrupt_mask(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, InterruptMaskState ims) {
	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_interrupt_mask(PI4IO_Instance, port, &current_port_config);

	if(ims == IT_MASK_DISABLE) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	PI4IO_set_port_interrupt_mask(PI4IO_Instance, port, current_port_config);

	return;
}

/* Interrupt Status READ-ONLY
 * 0 - Pin not source of interrupt
 * 1 - Pin is source of interrupt
 *
 * Default - 0
 */

HAL_StatusTypeDef PI4IO_get_port_interrupt_status(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_INTERRUPT_STATUS_PORT_0 + port, pvalue);

	return ret;
}

void PI4IO_get_pin_interrupt_status(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, uint8_t *const pvalue) {
	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_interrupt_status(PI4IO_Instance, port, &current_port_config);

	*pvalue = bitcheck(current_port_config, pin) > 0;

	return;
}

/* Interrupt Edge selection (Action required to generate interrupt, masking needs to be enabled)
 * 00 - Level change triggered (L->H or H->L)
 * 01 - Rising Edge triggered (L->H)
 * 10 - Falling edge triggered (H->L)
 * 11 - Any edge triggered (Rising or falling)
 *
 * Default - 0
 */

HAL_StatusTypeDef PI4IO_get_port_interrupt_edge_selection(PI4IOPtr const PI4IO_Instance, uint8_t port, uint16_t *const pins) {
	HAL_StatusTypeDef ret;

	uint8_t portA = 0x00;
	uint8_t portB = 0x00;
	uint8_t port_reg = (PI4IO_REG_INTERRUPT_EDGE_PORT_0A + (port * 2) ); // Determine which register corresponds to the given port

	ret = PI4IO_read_reg(PI4IO_Instance, port_reg, &portA);

	if(port != 4) {
		PI4IO_read_reg(PI4IO_Instance, port_reg+1, &portB);
	}

	*pins = portB;
	*pins <<= 8;
	*pins |= portA;

	return ret;


}

HAL_StatusTypeDef PI4IO_set_port_interrupt_edge_selection(PI4IOPtr const PI4IO_Instance, uint8_t port, uint16_t pins) {
	HAL_StatusTypeDef ret;

	uint8_t portA = (pins & 0xff);
	uint8_t portB = (pins & 0xff00) >> 8;
	uint8_t port_reg = (PI4IO_REG_INTERRUPT_EDGE_PORT_0A + (port * 2) ); // Determine which register corresponds to the given port

	ret = PI4IO_write_reg(PI4IO_Instance, port_reg, portA);
	PI4IO_write_reg(PI4IO_Instance, port_reg+1, portB);

	return ret;

}

void PI4IO_set_pin_interrupt_edge_selection(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, InterruptEdgeType iet) {
	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t port_reg = (PI4IO_REG_INTERRUPT_EDGE_PORT_0A + (port * 2) + (pin / 4));
	uint8_t current_port_config = 0x00;

	PI4IO_read_reg(PI4IO_Instance, port_reg, &current_port_config);

	uint8_t shiftamt = pin * 2 - ((pin > 3) * 8); // 2 bits per pin, 2 registers per port, we subtract 8 from shiftamt if the register is the second one for the port

	bitclear(current_port_config, shiftamt);
	bitclear(current_port_config, shiftamt+1);

	current_port_config |= (iet << shiftamt);


	PI4IO_write_reg(PI4IO_Instance, port_reg, current_port_config);

	return;

}


/* Interrupt clear registers WRITE-ONLY
 * 0 - Interrupt aready cleared
 * 1 - Writing this will reset corresponding interrupt source
 *
 * Default 0
 */

HAL_StatusTypeDef PI4IO_set_port_interrupt_clear(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_INTERRUPT_CLEAR_PORT_0 + port, pins);

	return ret;
}

void PI4IO_clear_all_interrupts(PI4IOPtr const PI4IO_Instance) {
	PI4IO_set_port_interrupt_clear(PI4IO_Instance, 0, 0xff);
	PI4IO_set_port_interrupt_clear(PI4IO_Instance, 1, 0xff);
	PI4IO_set_port_interrupt_clear(PI4IO_Instance, 2, 0xff);
	PI4IO_set_port_interrupt_clear(PI4IO_Instance, 3, 0xff);
	PI4IO_set_port_interrupt_clear(PI4IO_Instance, 4, 0xff);

	return;
}

/* Output Port Config - Should be programmed before configuring pin direction as output
 * 0 - Output Configured as push-pull
 * 1 - Output Configured as Open-drain
 *
 * Default - 0
 *
 * Individual pins can be configured using the next section of functions
 */

HAL_StatusTypeDef PI4IO_get_port_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_OUTPUT_PORT_CONFIG, pvalue);

	*pvalue = bitcheck(*pvalue, port) > 0;

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port, OutputPortConfig opc) {
	HAL_StatusTypeDef ret;
	uint8_t current_port_config;

	PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_OUTPUT_PORT_CONFIG, &current_port_config);

	if(opc == OPEN_DRAIN) {
		bitset(current_port_config, port);
	} else {
		bitclear(current_port_config, port);
	}

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_OUTPUT_PORT_CONFIG, current_port_config);

	return ret;
}


/* Individual pin output configuration
 * 0 - Output Configured as push-pull
 * 1 - Output Configured as Open-drain
 *
 * Default - 0
 */

HAL_StatusTypeDef PI4IO_get_port_pins_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_PIN_OUTPUT_CONFIG_PORT_0 + port, pvalue);

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_pins_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_PIN_OUTPUT_CONFIG_PORT_0 + port, pins);

	return ret;
}

void PI4IO_set_pin_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, OutputPortConfig opc) {
	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_pins_output_configuration(PI4IO_Instance, port, &current_port_config);

	if(opc == OPEN_DRAIN) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	PI4IO_set_port_pins_output_configuration(PI4IO_Instance, port, current_port_config);

	return;
}

/* Switch Debounce Enable - Can only be enabled on pins from ports 0,1
 * MUST CONFIGURE P2_0 AS INPUT TO ENABLE DEBOUNCE
 * 0 - Disabled
 * 1 - Enabled
 *
 * Default - 0
 *
 * Does not have any affect if pin is configured as output
 * */

HAL_StatusTypeDef PI4IO_get_port_debounce(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_DEBOUNCE_ENABLE_PORT_0 + port, pvalue);

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_debounce(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_DEBOUNCE_ENABLE_PORT_0 + port, pins);

	return ret;
}

void PI4IO_set_pin_debounce(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, DebounceState ds) {
	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_debounce(PI4IO_Instance, port, &current_port_config);

	if(ds == DEBOUNCE_ENABLE) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	PI4IO_set_port_debounce(PI4IO_Instance, port, current_port_config);

	return;
}

/* Switch Debounce count - Only for port 1
 * 0 - Input finally settles closed
 * 1 - Input finally settles open
 *
 * Default - 0
 */

HAL_StatusTypeDef PI4IO_get_port_debounce_count(PI4IOPtr const PI4IO_Instance, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_DEBOUNCE_COUNT, pvalue);

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_debounce_count(PI4IOPtr const PI4IO_Instance, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_DEBOUNCE_COUNT, pins);

	return ret;
}

void PI4IO_set_pin_debounce_count(PI4IOPtr const PI4IO_Instance, uint8_t pin, DebounceCount dc) {
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_debounce_count(PI4IO_Instance, &current_port_config);

	if(dc == SETTLE_OPEN) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	PI4IO_set_port_debounce_count(PI4IO_Instance, current_port_config);

	return;
}

/* Writing to output pins
 * 0 - Pin low
 * 1 - Pin High
 *
 * Default - 1
 */

HAL_StatusTypeDef PI4IO_get_port_output_state(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_read_reg(PI4IO_Instance, PI4IO_REG_OUTPUT_PORT_0 + port, pvalue);

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_output_state(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_OUTPUT_PORT_0 + port, pins);

	return ret;
}

HAL_StatusTypeDef PI4IO_set_port_output_state_struct(PI4IOPtr const PI4IO_Instance, PortStates *const ps) {
	HAL_StatusTypeDef ret;

	ret = PI4IO_write_reg(PI4IO_Instance, PI4IO_REG_OUTPUT_PORT_0 + ps->port, ps);

	return ret;
}

HAL_StatusTypeDef PI4IO_write_pin(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, PinState ps) {
	HAL_StatusTypeDef ret;

	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);
	uint8_t current_port_config = 0x00;

	PI4IO_get_port_output_state(PI4IO_Instance, port, &current_port_config);

	if(ps == PIN_HIGH) {
		bitset(current_port_config, pin);
	} else {
		bitclear(current_port_config, pin);
	}

	ret = PI4IO_set_port_output_state(PI4IO_Instance, port, current_port_config);

	return ret;
}


/*
 * Reading Input pin states
 * Will clear all pending interrupts
 * Will return 0 always for pins configured as open-drain output
 */
HAL_StatusTypeDef PI4IO_poll_pin(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	uint8_t port = extractport(port_pin);
	uint8_t pin = extractpin(port_pin);

	uint8_t port_reg = PI4IO_REG_INPUT_PORT_0 + port;

	ret = PI4IO_read_reg(PI4IO_Instance, port_reg, pvalue);

	*pvalue = bitcheck(*pvalue, pin) > 0;

	return ret;
}

/*
 * Will clear all pending interrupts
 * Will return 0 always for pins configured as open-drain output
 */
HAL_StatusTypeDef PI4IO_poll_port(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue) {
	HAL_StatusTypeDef ret;

	uint8_t port_reg = PI4IO_REG_INPUT_PORT_0 + port;

	ret = PI4IO_read_reg(PI4IO_Instance, port_reg, pvalue);

	return ret;
}

HAL_StatusTypeDef PI4IO_poll_port_struct(PI4IOPtr const PI4IO_Instance, PortStates *const portVals) {
	HAL_StatusTypeDef ret;

	uint8_t port_reg = PI4IO_REG_INPUT_PORT_0 + portVals->port;

	ret = PI4IO_read_reg(PI4IO_Instance, port_reg, portVals);

	return ret;
}

HAL_StatusTypeDef PI4IO_get_port_interrupt_status_struct(PI4IOPtr const PI4IO_Instance, PortStates *const portVals) {
	HAL_StatusTypeDef ret;

	uint8_t port_reg = PI4IO_REG_INTERRUPT_STATUS_PORT_0 + portVals->port;

	ret = PI4IO_read_reg(PI4IO_Instance, port_reg, portVals);

	return ret;
}


