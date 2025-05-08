
add_library(rp_library SHARED
  grid_map.cpp
  draw_helpers.cpp
  dmap.cpp
  dmap_localizer.cpp)

target_link_libraries(rp_library
  ${OpenCV_LIBS}
  )

add_executable(test1.cpp
  test1)

