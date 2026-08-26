cmake_minimum_required(VERSION 3.16)

# ============================================================================
# tools -- utility layer, built as an OBJECT library.
#
# IMPORTANT: this library declares NO include directories of its own.
# Its sources include <Eigen/Dense>, <opencv2/...>, <fmt/...>, <spdlog/...>
# and nlohmann/json.hpp, but those headers resolve ONLY because the root
# CMakeLists.txt runs find_package(...) and include_directories(...) at root
# scope BEFORE add_subdirectory(tools). Therefore this directory MUST NOT be
# configured or built standalone -- always build from the project root.
#
# Sources are enumerated explicitly. Do not switch to file(GLOB) or
# aux_source_directory: the build set must stay predictable and reviewable.
# Only .cpp files are listed here. Header-only helpers such as yaml.hpp and
# thread_safe_queue.hpp have no matching .cpp and must not appear below.
# ============================================================================

add_library(tools OBJECT
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

# Make two previously implicit dependencies explicit:
#   spdlog        -> used by logger.cpp / logger.hpp
#   nlohmann_json -> used by plotter.hpp
# The original project called find_package for both but never linked them.
# That works only where the distro ships a header-only path; on a distro where
# spdlog is a compiled library it would fail to link. Linking explicitly costs
# nothing and removes that fragility.
target_link_libraries(tools PUBLIC spdlog::spdlog nlohmann_json::nlohmann_json)
