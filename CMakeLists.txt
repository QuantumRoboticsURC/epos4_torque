cmake_minimum_required(VERSION 3.18)
project(epos4_torque LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(eposlib REQUIRED)

# La red CANopen de este paquete (EPOS4 en el nodo 5), convertida en el DCF
# del maestro al compilar. Mapea Target torque (0x6071) y Torque actual
# (0x6077) en los PDO, que es lo que necesita el modo cíclico.
eposlib_generate_dcf(red_nodo5
  DESTINATION share/${PROJECT_NAME}/config)

set(DEFAULT_DCF ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/red_nodo5/master.dcf)

foreach(programa control_torque info_motor)
  add_executable(${programa} src/${programa}.cpp)
  target_link_libraries(${programa} eposlib::eposlib)
  target_compile_definitions(${programa} PRIVATE
    DEFAULT_DCF="${DEFAULT_DCF}" DEFAULT_NODE=5)
  install(TARGETS ${programa} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
