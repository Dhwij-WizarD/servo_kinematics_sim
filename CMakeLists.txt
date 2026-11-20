cmake_minimum_required(VERSION 3.16)
project(simple_mujoco_ik CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(glfw3 REQUIRED)

# Automatically locate MuJoCo headers & library inside the active Python environment
execute_process(
    COMMAND python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
    OUTPUT_VARIABLE MUJOCO_PY_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(${MUJOCO_PY_DIR}/include)
link_directories(${MUJOCO_PY_DIR})

add_executable(sim src/main.cpp)
target_link_libraries(sim mujoco glfw)

# Embed RPATH so ./sim finds libmujoco.so at runtime without LD_LIBRARY_PATH hacks
set_target_properties(sim PROPERTIES
    BUILD_RPATH "${MUJOCO_PY_DIR}"
    INSTALL_RPATH "${MUJOCO_PY_DIR}"
)