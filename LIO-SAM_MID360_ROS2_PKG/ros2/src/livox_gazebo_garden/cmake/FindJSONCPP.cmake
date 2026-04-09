if(NOT TARGET JsonCpp::JsonCpp)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(JSONCPP jsoncpp IMPORTED_TARGET)
  
  if(JSONCPP_FOUND)
    add_library(JsonCpp::JsonCpp ALIAS PkgConfig::JSONCPP)
    set(JSONCPP_FOUND TRUE)
  else()
    find_path(JSONCPP_INCLUDE_DIRS json/json.h PATH_SUFFIXES jsoncpp)
    find_library(JSONCPP_LIBRARIES NAMES jsoncpp)
    
    if(JSONCPP_INCLUDE_DIRS AND JSONCPP_LIBRARIES)
      add_library(JsonCpp::JsonCpp SHARED IMPORTED)
      set_target_properties(JsonCpp::JsonCpp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${JSONCPP_INCLUDE_DIRS}"
        IMPORTED_LOCATION "${JSONCPP_LIBRARIES}"
      )
      set(JSONCPP_FOUND TRUE)
    endif()
  endif()
else()
  set(JSONCPP_FOUND TRUE)
endif()
