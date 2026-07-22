# Define the prebuilt micro-ROS static library as a proper CMake target
add_library(microros STATIC IMPORTED GLOBAL)

# Point that target at the actual .a file on disk
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/libmicroros/libmicroros.a
)

# Attach the include directory to the target itself, so anything that
# links against "microros" automatically gets the right headers
target_include_directories(microros INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)