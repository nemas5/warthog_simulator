cmake_minimum_required(VERSION 3.15)
project(PSPSO)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PSPSO_ARTIFACT_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Directory for PSPSO libraries and Python module")
file(MAKE_DIRECTORY "${PSPSO_ARTIFACT_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PSPSO_ARTIFACT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PSPSO_ARTIFACT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PSPSO_ARTIFACT_DIR}")

# Поиск Python и pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# --- Библиотека Particle ---
add_library(Particle STATIC
    Particle/Particle.cpp
)
target_include_directories(Particle PUBLIC ${CMAKE_SOURCE_DIR})

# --- Библиотека Subsworm ---
add_library(Subsworm STATIC
    Subsworm/Subsworm.cpp
)
target_include_directories(Subsworm PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(Subsworm PUBLIC Particle)

# --- Библиотека PSO ---
add_library(PSO STATIC
    PSO/PSO.cpp
)
target_include_directories(PSO PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(PSO PUBLIC Subsworm Particle)

# --- pybind11-модуль ---
pybind11_add_module(pspso
    bindings.cpp   # файл с PYBIND11_MODULE(...)
)
target_link_libraries(pspso PRIVATE PSO)
target_include_directories(pspso PRIVATE ${CMAKE_SOURCE_DIR})
