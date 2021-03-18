find_package(idocp REQUIRED)

add_executable(
    posture
    posture.cpp
)
target_include_directories(
    posture
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${IDOCP_INCLUDE_DIR}
)
target_link_libraries(
    posture
    PRIVATE
    ${PROJECT_NAME}
    idocp::idocp
)