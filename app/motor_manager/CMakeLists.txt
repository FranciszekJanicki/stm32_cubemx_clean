add_library(motor_manager STATIC)

add_subdirectory(${APP_DIR}/motor_manager/a4988)
add_subdirectory(${APP_DIR}/motor_manager/step_motor)
add_subdirectory(${APP_DIR}/motor_manager/pid_regulator)
add_subdirectory(${APP_DIR}/motor_manager/rotary_encoder)
add_subdirectory(${APP_DIR}/motor_manager/motor_driver)

target_sources(motor_manager PRIVATE 
    motor_manager.c
)

target_include_directories(motor_manager PUBLIC
   ${APP_DIR}/motor_manager/motor_driver
   ${APP_DIR}/motor_manager/a4988
   ${APP_DIR}/motor_manager/step_motor
   ${APP_DIR}/motor_manager/pid_regulator
   ${APP_DIR}/motor_manager/rotary_encoder
)

target_link_libraries(motor_manager PUBLIC
    motor_driver
    a4988
    step_motor
    pid_regulator
    rotary_encoder
    stm32cubemx
)

target_compile_options(motor_manager PUBLIC
    -std=c23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -Wformat=2
    -Wformat-security
    -Wmissing-prototypes
    -Wmissing-declarations
    -Wstrict-prototypes
    -Wold-style-definition
    -Wundef
    -Wvla
    -Wpointer-arith
    -Wstrict-aliasing=2
)