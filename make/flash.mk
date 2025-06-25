include make/common.mk

.PHONY: flash_uart
flash_uart: 
	STM32_Programmer_CLI -c port=swd -d $(PROJECT_BINARY) -rst

.PHONY: flash_usb
flash_usb:
	STM32_Programmer_CLI -c port=$(USB_PORT) -d $(PROJECT_BINARY) -rst

BAUD_RATE := 115200

.PHONY: monitor_uart
monitor_uart:
	minicom -D $(UART_PORT) -b $(BAUD_RATE)

.PHONY: monitor_usb
monitor_usb:
	minicom -D $(USB_PORT) -b $(BAUD_RATE)