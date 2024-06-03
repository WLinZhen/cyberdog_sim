#add_library(inih STATIC ini.c ini.h INIReader.h INIReader.cpp)
add_library(inih SHARED ini.c ini.h INIReader.h INIReader.cpp)
install(TARGETS inih
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)