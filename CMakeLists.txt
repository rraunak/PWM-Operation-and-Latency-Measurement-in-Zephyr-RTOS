set(IS_TEST 1)

cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(project1_45)

FILE(GLOB app_sources src/main.c)
target_sources(app PRIVATE ${app_sources})
target_compile_options(app PRIVATE -Wall)
