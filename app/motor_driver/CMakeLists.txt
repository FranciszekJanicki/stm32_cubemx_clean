add_library(motor_driver STATIC)

target_sources(motor_driver PRIVATE 
    "motor_driver.c"
)

target_include_directories(motor_driver PRIVATE
    ${APP_DIR}/pid_regulator
    ${APP_DIR}/step_motor
    ${APP_DIR}/rotary_encoder
)

target_link_libraries(motor_driver PRIVATE
    step_motor
    pid_regulator
    rotary_encoder
)

target_compile_options(motor_driver PUBLIC
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