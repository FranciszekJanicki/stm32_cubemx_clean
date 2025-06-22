add_library(joint_task STATIC)

add_subdirectory(${APP_DIR}/joint_task/a4988)
add_subdirectory(${APP_DIR}/joint_task/step_motor)
add_subdirectory(${APP_DIR}/joint_task/pid_regulator)
add_subdirectory(${APP_DIR}/joint_task/rotary_encoder)
add_subdirectory(${APP_DIR}/joint_task/motor_driver)

target_sources(joint_task PRIVATE 
    joint_task.c
    joint_manager.c
)

target_include_directories(joint_task PUBLIC
   ${APP_DIR}/joint_task/motor_driver
   ${APP_DIR}/joint_task/a4988
   ${APP_DIR}/joint_task/step_motor
   ${APP_DIR}/joint_task/pid_regulator
   ${APP_DIR}/joint_task/rotary_encoder
   ${CMAKE_DIR}/stm32cubemx
)

target_link_libraries(joint_task PUBLIC
    motor_driver
    a4988
    step_motor
    pid_regulator
    rotary_encoder
    stm32cubemx
)

target_compile_options(joint_task PUBLIC
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