find_package(CGAL QUIET COMPONENTS Core )
link_directories(${CGAL_LIBRARY_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})
message(STATUS "CGAL found (required for some tests): ${CGAL_FOUND}")
find_package(GMP REQUIRED)
link_directories(${GMP_LIBRARY_DIRS})
include_directories(${GMP_INCLUDE_DIR})
message(STATUS "GMP found (required for some tests): ${GMP_FOUND}")
find_package(MPFR REQUIRED)
include_directories(${MPFR_INCLUDE_DIR})
link_directories(${MPFR_LIBRARY_DIRS})
message(STATUS "MPFR found (required for some tests): ${MPFR_FOUND}")