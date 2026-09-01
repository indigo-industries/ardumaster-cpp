add_library(ap-vehicle INTERFACE)
target_include_directories(ap-vehicle INTERFACE include)
target_compile_features(ap-vehicle INTERFACE cxx_std_20)
target_link_libraries(ap-vehicle INTERFACE ap-hal ap-rc-channel ap-srv-channel ap-ahrs ap-gps ap-compass ap-airspeed ap-baro ap-fw-control ap-steer-control ap-tecs ap-math ap-nav ap-common)
