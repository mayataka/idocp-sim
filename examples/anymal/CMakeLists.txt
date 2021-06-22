find_package(idocp REQUIRED)

add_executable(
    posture_control
    posture_control.cpp
    posture_cost.cpp
)
target_include_directories(
    posture_control
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${IDOCP_INCLUDE_DIR}
)
target_link_libraries(
    posture_control
    PRIVATE
    ${PROJECT_NAME}
    idocp::idocp
)