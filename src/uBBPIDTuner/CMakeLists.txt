#--------------------------------------------------------
# CMakeLists.txt for the pymoos2 app:           uBBPIDTuner
# Installs the Python sources to bin/ alongside the C++ apps,
# so pAntler finds `uBBPIDTuner` on PATH.
#--------------------------------------------------------
set(APP_NAME uBBPIDTuner)

# Syntax-check Python sources at build time
add_custom_target(${APP_NAME}_check
    COMMAND python3 -m py_compile ${CMAKE_CURRENT_SOURCE_DIR}/${APP_NAME}.py
    COMMAND python3 -m py_compile ${CMAKE_CURRENT_SOURCE_DIR}/BBPIDTuner.py
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/__pycache__
    COMMENT "Syntax-checking ${APP_NAME} Python sources"
)

add_custom_target(${APP_NAME} ALL
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/${APP_NAME}.py
        ${EXECUTABLE_OUTPUT_PATH}/${APP_NAME}
    COMMAND chmod +x ${EXECUTABLE_OUTPUT_PATH}/${APP_NAME}
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/BBPIDTuner.py
        ${EXECUTABLE_OUTPUT_PATH}/BBPIDTuner.py
    COMMENT "Installing ${APP_NAME} to bin/"
    DEPENDS ${APP_NAME}_check
)
