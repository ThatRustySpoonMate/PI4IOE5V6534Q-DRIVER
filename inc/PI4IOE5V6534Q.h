/*
 * PI4IOE5V6534Q.h
 *
 *  Created on: 15 Jul. 2023
 *      Author: Ethan
 */

#include "main.h"

#ifndef INC_PI4IOE5V6534Q_H_
#define INC_PI4IOE5V6534Q_H_

#define PORTCOUNT 5

typedef struct PI4IO* PI4IOPtr;

struct PI4IO {
	I2C_HandleTypeDef* hi2cx;
	uint8_t i2c_addr;
};

/* I2C Addresses */
#define PI4IO_ADDR_SCL 0x20 // ADDR -> SCL
#define PI4IO_ADDR_SDA 0x21 // ADDR -> SDA
#define PI4IO_ADDR_VSS 0x22 // ADDR -> VSS
#define PI4IO_ADDR_VDD 0x23 // ADDR -> VDD

/* Register Commands */
#define PI4IO_CMD_READ_REG 1
#define PI4IO_CMD_WRITE_REG 0
#define PI4IO_CMD_RESET 0x06

/* Ports and Pins */
#define PIN(port, pin) ((port << 4) | pin)
#define extractport(port_pin) (port_pin >> 4)
#define extractpin(port_pin) (port_pin & 0x0f)

//#define PI4IO_DIRECTION_OUTPUT 0
//#define PI4IO_DIRECTION_INPUT 1
#define PI4IO_PORT_0 0
#define PI4IO_PORT_1 1
#define PI4IO_PORT_2 2
#define PI4IO_PORT_3 3
#define PI4IO_PORT_4 4


/* Register Addresses */
#define PI4IO_REG_INPUT_PORT_0 0x00
#define PI4IO_REG_INPUT_PORT_1 0x01
#define PI4IO_REG_INPUT_PORT_2 0x02
#define PI4IO_REG_INPUT_PORT_3 0x03
#define PI4IO_REG_INPUT_PORT_4 0x04

#define PI4IO_REG_OUTPUT_PORT_0 0x05
#define PI4IO_REG_OUTPUT_PORT_1 0x06
#define PI4IO_REG_OUTPUT_PORT_2 0x07
#define PI4IO_REG_OUTPUT_PORT_3 0x08
#define PI4IO_REG_OUTPUT_PORT_4 0x09

#define PI4IO_REG_POLARITY_INVERSION_PORT_0 0x0a
#define PI4IO_REG_POLARITY_INVERSION_PORT_1 0x0b
#define PI4IO_REG_POLARITY_INVERSION_PORT_2 0x0c
#define PI4IO_REG_POLARITY_INVERSION_PORT_3 0x0d
#define PI4IO_REG_POLARITY_INVERSION_PORT_4 0x0e

#define PI4IO_REG_CONFIG_PORT_0 0x0f
#define PI4IO_REG_CONFIG_PORT_1 0x10
#define PI4IO_REG_CONFIG_PORT_2 0x11
#define PI4IO_REG_CONFIG_PORT_3 0x12
#define PI4IO_REG_CONFIG_PORT_4 0x13

#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_0A 0x30
#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_0B 0x31
#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_1A 0x32
#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_1B 0x33
#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_2A 0x34
#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_2B 0x35
#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_3A 0x36
#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_3B 0x37
#define PI4IO_REG_OUTPUT_DRIVE_LEVEL_4A 0x38

#define PI4IO_REG_INPUT_LATCH_PORT_0 0x3A
#define PI4IO_REG_INPUT_LATCH_PORT_1 0x3B
#define PI4IO_REG_INPUT_LATCH_PORT_2 0x3C
#define PI4IO_REG_INPUT_LATCH_PORT_3 0x3D
#define PI4IO_REG_INPUT_LATCH_PORT_4 0x3E

#define PI4IO_REG_PULLUP_PULLDOWN_ENABLE_PORT_0 0x3F
#define PI4IO_REG_PULLUP_PULLDOWN_ENABLE_PORT_1 0x40
#define PI4IO_REG_PULLUP_PULLDOWN_ENABLE_PORT_2 0x41
#define PI4IO_REG_PULLUP_PULLDOWN_ENABLE_PORT_3 0x42
#define PI4IO_REG_PULLUP_PULLDOWN_ENABLE_PORT_4 0x43

#define PI4IO_REG_PULLUP_PULLDOWN_SELECT_PORT_0 0x44
#define PI4IO_REG_PULLUP_PULLDOWN_SELECT_PORT_1 0x45
#define PI4IO_REG_PULLUP_PULLDOWN_SELECT_PORT_2 0x46
#define PI4IO_REG_PULLUP_PULLDOWN_SELECT_PORT_3 0x47
#define PI4IO_REG_PULLUP_PULLDOWN_SELECT_PORT_4 0x48

#define PI4IO_REG_INTERRUPT_MASK_PORT_0 0x49
#define PI4IO_REG_INTERRUPT_MASK_PORT_1 0x4A
#define PI4IO_REG_INTERRUPT_MASK_PORT_2 0x4B
#define PI4IO_REG_INTERRUPT_MASK_PORT_3 0x4C
#define PI4IO_REG_INTERRUPT_MASK_PORT_4 0x4D

#define PI4IO_REG_INTERRUPT_STATUS_PORT_0 0x4E
#define PI4IO_REG_INTERRUPT_STATUS_PORT_1 0x4F
#define PI4IO_REG_INTERRUPT_STATUS_PORT_2 0x50
#define PI4IO_REG_INTERRUPT_STATUS_PORT_3 0x51
#define PI4IO_REG_INTERRUPT_STATUS_PORT_4 0x52

#define PI4IO_REG_OUTPUT_PORT_CONFIG 0x53

#define PI4IO_REG_INTERRUPT_EDGE_PORT_0A 0x54
#define PI4IO_REG_INTERRUPT_EDGE_PORT_0B 0x55
#define PI4IO_REG_INTERRUPT_EDGE_PORT_1A 0x56
#define PI4IO_REG_INTERRUPT_EDGE_PORT_1B 0x57
#define PI4IO_REG_INTERRUPT_EDGE_PORT_2A 0x58
#define PI4IO_REG_INTERRUPT_EDGE_PORT_2B 0x59
#define PI4IO_REG_INTERRUPT_EDGE_PORT_3A 0x5A
#define PI4IO_REG_INTERRUPT_EDGE_PORT_3B 0x5B
#define PI4IO_REG_INTERRUPT_EDGE_PORT_4A 0x5C

#define PI4IO_REG_INTERRUPT_CLEAR_PORT_0 0x5E
#define PI4IO_REG_INTERRUPT_CLEAR_PORT_1 0x5F
#define PI4IO_REG_INTERRUPT_CLEAR_PORT_2 0x60
#define PI4IO_REG_INTERRUPT_CLEAR_PORT_3 0x61
#define PI4IO_REG_INTERRUPT_CLEAR_PORT_4 0x62

#define PI4IO_REG_INPUT_STATUS_PORT_0 0x63
#define PI4IO_REG_INPUT_STATUS_PORT_1 0x64
#define PI4IO_REG_INPUT_STATUS_PORT_2 0x65
#define PI4IO_REG_INPUT_STATUS_PORT_3 0x66
#define PI4IO_REG_INPUT_STATUS_PORT_4 0x67

#define PI4IO_REG_PIN_OUTPUT_CONFIG_PORT_0 0x68
#define PI4IO_REG_PIN_OUTPUT_CONFIG_PORT_1 0x69
#define PI4IO_REG_PIN_OUTPUT_CONFIG_PORT_2 0x6A
#define PI4IO_REG_PIN_OUTPUT_CONFIG_PORT_3 0x6B
#define PI4IO_REG_PIN_OUTPUT_CONFIG_PORT_4 0x6C

#define PI4IO_REG_DEBOUNCE_ENABLE_PORT_0 0x6D
#define PI4IO_REG_DEBOUNCE_ENABLE_PORT_1 0x6E

#define PI4IO_REG_DEBOUNCE_COUNT 0x6F


/* Macros */
typedef enum {
	OUTPUT,
	INPUT
}PinDirection;

typedef enum {
	PIN_LOW,
	PIN_HIGH
}PinState;

typedef enum  {
	OUTPUT_DRIVE_0x25, // 25%  Output current
	OUTPUT_DRIVE_0x50, // 50%  Output current
	OUTPUT_DRIVE_0x75, // 75%  Output current
	OUTPUT_DRIVE_1x00  // 100% Output current
} OutputDriveLevel;

typedef enum {
	INPUT_POLARITY_NORMAL,
	INPUT_POLARITY_INVERTED
} InputPolarity;

typedef enum {
	INPUT_UNLATCHED,
	INPUT_LATCHED
} InputLatchState;

typedef enum {
	RESISTOR_DISABLE,
	RESISTOR_ENABLE
}ResistorState;

typedef enum {
	PULLDOWN_100k,
	PULLUP_100k
}ResistorType;

typedef enum {
	IT_MASK_ENABLE,
	IT_MASK_DISABLE
}InterruptMaskState;

typedef enum {
	IT_NOT_SOURCE,
	IT_SOURCE
}InterruptStatus;

typedef enum {
	LEVEL_CHANGE,
	EDGE_RISING,
	EDGE_FALLING,
	EDGE_RISING_FALLING
}InterruptEdgeType;

typedef enum {
	PUSH_PULL,
	OPEN_DRAIN
}OutputPortConfig;

typedef enum {
	DEBOUNCE_DISABLE,
	DEBOUNCE_ENABLE
}DebounceState;

typedef enum {
	SETTLE_CLOSED,
	SETTLE_OPEN
}DebounceCount;

/*
 * Struct that stores the state of each pin in a port
 * Can be used to store inputs or outputs via the following functions:
 *  - PI4IO_set_port_output_state_struct
 *  - PI4IO_poll_port_struct
 * Also holds the port number
 */
typedef struct {
	uint8_t pin_0   : 1;
	uint8_t pin_1   : 1;
	uint8_t pin_2   : 1;
	uint8_t pin_3   : 1;
	uint8_t pin_4   : 1;
	uint8_t pin_5   : 1;
	uint8_t pin_6   : 1;
	uint8_t pin_7   : 1;

	uint8_t port    : 3;
} PortStates;



/* Declare functions */
HAL_StatusTypeDef PI4IO_read_reg(PI4IOPtr const PI4IO_Instance, uint8_t reg_num, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_write_reg(PI4IOPtr const PI4IO_Instance, uint8_t reg_num, uint8_t value);
HAL_StatusTypeDef PI4IO_soft_reset(PI4IOPtr const PI4IO_Instance);
HAL_StatusTypeDef PI4IO_get_port_pins_direction(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_pins_direction(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
HAL_StatusTypeDef PI4IO_set_single_pin_direction(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, uint8_t direction);
void PI4IO_set_input_port_polarity_normal(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
void PI4IO_set_input_port_polarity_inverted(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
HAL_StatusTypeDef PI4IO_set_input_pin_polarity(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, InputPolarity ip);
HAL_StatusTypeDef PI4IO_get_input_port_polarity(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_get_output_pin_drive_level(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_output_pin_drive_level(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, OutputDriveLevel odl);
void PI4IO_set_output_port_drive_level(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins[2]);
HAL_StatusTypeDef PI4IO_get_port_input_latch_state(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_input_latch_state(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
void PI4IO_set_pin_input_latch_state(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, InputLatchState ls);
HAL_StatusTypeDef PI4IO_get_port_resistor_enable(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_resistor_enable(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
void PI4IO_set_pin_resistor_enable(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, ResistorState rs);
HAL_StatusTypeDef PI4IO_get_port_resistor_selection(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_resistor_selection(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
void setPI4IO_set_pin_resistor_selection(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, ResistorType rt);
HAL_StatusTypeDef PI4IO_get_port_interrupt_mask(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_interrupt_mask(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
void PI4IO_set_pin_interrupt_mask(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, InterruptMaskState ims);
HAL_StatusTypeDef PI4IO_get_port_interrupt_status(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
void PI4IO_get_pin_interrupt_status(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_get_port_interrupt_edge_selection(PI4IOPtr const PI4IO_Instance, uint8_t port, uint16_t *const pins);
HAL_StatusTypeDef PI4IO_set_port_interrupt_edge_selection(PI4IOPtr const PI4IO_Instance, uint8_t port, uint16_t pins);
void PI4IO_set_pin_interrupt_edge_selection(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, InterruptEdgeType iet);
HAL_StatusTypeDef PI4IO_set_port_interrupt_clear(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
void PI4IO_clear_all_interrupts(PI4IOPtr const PI4IO_Instance);
HAL_StatusTypeDef PI4IO_get_port_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port, OutputPortConfig opc);
HAL_StatusTypeDef PI4IO_get_port_pins_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_pins_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
void PI4IO_set_pin_output_configuration(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, OutputPortConfig opc);
HAL_StatusTypeDef PI4IO_get_port_debounce(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_debounce(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
void PI4IO_set_pin_debounce(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, DebounceState ds);
HAL_StatusTypeDef PI4IO_get_port_debounce_count(PI4IOPtr const PI4IO_Instance, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_debounce_count(PI4IOPtr const PI4IO_Instance, uint8_t pins);
void PI4IO_set_pin_debounce_count(PI4IOPtr const PI4IO_Instance, uint8_t pin, DebounceCount dc);
HAL_StatusTypeDef PI4IO_get_port_output_state(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_set_port_output_state_struct(PI4IOPtr const PI4IO_Instance, PortStates *const ps);
HAL_StatusTypeDef PI4IO_set_port_output_state(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t pins);
HAL_StatusTypeDef PI4IO_write_pin(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, PinState ps);
HAL_StatusTypeDef PI4IO_poll_port(PI4IOPtr const PI4IO_Instance, uint8_t port, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_poll_pin(PI4IOPtr const PI4IO_Instance, uint8_t port_pin, uint8_t *const pvalue);
HAL_StatusTypeDef PI4IO_poll_port_struct(PI4IOPtr const PI4IO_Instance, PortStates *const portVals);
HAL_StatusTypeDef PI4IO_get_port_interrupt_status_struct(PI4IOPtr const PI4IO_Instance, PortStates *const portVals);



#endif /* INC_PI4IOE5V6534Q_H_ */
