add_executable(main)

target_sources(main PRIVATE 
    main.c
    nvic.c
)

target_include_directories(main PRIVATE 
    ${CMAKE_DIR}/stm32cubemx
    ${APP_DIR}/motor_driver
    ${APP_DIR}/a4988
    ${APP_DIR}/step_motor
    ${APP_DIR}/pid_regulator
    ${APP_DIR}/rotary_encoder
)

target_link_libraries(main PRIVATE
    stm32cubemx
    motor_driver
    a4988
    step_motor
    pid_regulator
    rotary_encoder
)

target_compile_options(main PUBLIC
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
