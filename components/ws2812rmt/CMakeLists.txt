idf_component_register(REQUIRES log driver)

target_include_directories(${COMPONENT_TARGET} INTERFACE
    "${COMPONENT_DIR}/inc"
)

target_sources(${COMPONENT_TARGET} INTERFACE
    "${COMPONENT_DIR}/src/ws2812_control.c"
)
