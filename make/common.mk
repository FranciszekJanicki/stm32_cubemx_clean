PROJECT_DIR := $(shell pwd)
APP_DIR := $(PROJECT_DIR)/app
MAIN_DIR := $(APP_DIR)/main
COMPONENTS_DIR := $(APP_DIR)/components
THIRD_PARTY_DIR := $(APP_DIR)/third_party
CORE_DIR := $(PROJECT_DIR)/Core
CMAKE_DIR := $(PROJECT_DIR)/cmake
BUILD_DIR := $(PROJECT_DIR)/build
DRIVERS_DIR := $(PROJECT_DIR)/Drivers
SCRIPTS_DIR := $(PROJECT_DIR)/scripts

PROJECT_NAME := project
PROJECT_BINARY := $(BUILD_DIR)/app/main/main.elf

UART_PORT := /dev/ttyACM0
USB_PORT := /dev/ttyACM0