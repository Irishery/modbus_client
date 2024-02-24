cmake_minimum_required(VERSION 3.15) # Проверка версии CMake.
									 # Если версия установленой программы
									 # старее указаной, произайдёт аварийный выход.

set(PROJECT_NAME modbus_client) # задать значение PROJECT_NAME		 
project("${PROJECT_NAME}")           # установить имя проекта


set(CMAKE_CXX_STANDARD 17)			 # устанавливаем 17 стандарт языка
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)


# Подключить header-файлы
include_directories("headers/")
include_directories("libs/libmodbus-3.1.10/src/")
# Сказать программе, что должен быть исполняемый файл
# file(GLOB SOURCES "src/*.cpp")
# file(GLOB LIBS "src/*.h")

set(SOURCES
    src/main.cpp)

set(HEADERS
    headers/entities.h)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC headers)


#target_include_directories("${PROJECT_NAME}" PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/headers/)

target_link_directories("${PROJECT_NAME}" PUBLIC "libs/libmodbus-3.1.10/src/.libs/")
target_link_libraries("${PROJECT_NAME}" "modbus")

# Копирование файла
#file(COPY "input.txt" DESTINATION "/")
