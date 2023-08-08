file(GLOB_RECURSE ALL_SRCS "*.cpp")
add_library(myslam ${ALL_SRCS})

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
