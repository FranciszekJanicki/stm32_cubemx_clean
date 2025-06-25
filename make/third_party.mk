include make/common.mk

PID_REGULATOR_DIR := $(THIRD_PARTY_DIR)/pid_regulator
PID_REGULATOR_SSH := git@github.com:FranciszekJanicki/pid_regulator.git

STEP_MOTOR_DIR := $(THIRD_PARTY_DIR)/step_motor
STEP_MOTOR_SSH := git@github.com:FranciszekJanicki/step_motor.git

MOTOR_DRIVER_DIR := $(THIRD_PARTY_DIR)/motor_driver
MOTOR_DRIVER_SSH := git@github.com:FranciszekJanicki/motor_driver.git

A4988_DIR := $(THIRD_PARTY_DIR)/a4988
A4988_SSH := git@github.com:FranciszekJanicki/a4988.git

ROTARY_ENCODER_DIR := $(THIRD_PARTY_DIR)/rotary_encoder
A4988_SSH := git@github.com:FranciszekJanicki/a4988.git

ADD_SCRIPT := ./scripts/add_submodule.sh
REMOVE_SCRIPT := ./scripts/remove_submodule.sh

.PHONY: add_pid_regulator
add_pid_regulator:
	@$(ADD_SCRIPT) $(PID_REGULATOR_SSH) $(PID_REGULATOR_DIR)

.PHONY: remove_pid_regulator
remove_pid_regulator:
	@$(REMOVE_SCRIPT) $(PID_REGULATOR_DIR)

.PHONY: add_step_motor
add_step_motor:
	@$(ADD_SCRIPT) $(STEP_MOTOR_SSH) $(STEP_MOTOR_DIR)

.PHONY: remove_step_motor
remove_step_motor:
	@$(REMOVE_SCRIPT) $(STEP_MOTOR_DIR)

.PHONY: add_motor_driver
add_motor_driver:
	@$(ADD_SCRIPT) $(MOTOR_DRIVER_SSH) $(MOTOR_DRIVER_DIR)

.PHONY: remove_motor_driver
remove_motor_driver:
	@$(REMOVE_SCRIPT) $(MOTOR_DRIVER_DIR)

.PHONY: add_a4988
add_a4988:
	@$(ADD_SCRIPT) $(A4988_SSH) $(A4988_DIR)

.PHONY: remove_a4988
remove_a4988:
	@$(REMOVE_SCRIPT) $(A4988_DIR)

.PHONY: add_third_party
add_third_party: add_pid_regulator add_step_motor add_motor_driver add_a4988

.PHONY: remove_third_party
remove_third_party: remove_pid_regulator remove_step_motor remove_motor_driver remove_a4988
