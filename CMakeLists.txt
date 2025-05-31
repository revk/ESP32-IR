cmake_minimum_required(VERSION 3.5...3.25)

set(SOURCES "ir.c")
set(RECS "driver" "ESP32-RevK")

idf_component_register(
			SRCS ${SOURCES}
			INCLUDE_DIRS "include"
			REQUIRES ${RECS}
)

add_definitions(-DBUILD_ESP32_USING_CMAKE)
