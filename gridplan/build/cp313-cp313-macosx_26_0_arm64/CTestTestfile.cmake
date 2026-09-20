# CMake generated Testfile for 
# Source directory: /Users/Dylan/Documents/autonomy_planning/gridplan
# Build directory: /Users/Dylan/Documents/autonomy_planning/gridplan/build/cp313-cp313-macosx_26_0_arm64
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test_grid]=] "/Users/Dylan/Documents/autonomy_planning/gridplan/build/cp313-cp313-macosx_26_0_arm64/test_grid")
set_tests_properties([=[test_grid]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/Dylan/Documents/autonomy_planning/gridplan/CMakeLists.txt;55;add_test;/Users/Dylan/Documents/autonomy_planning/gridplan/CMakeLists.txt;0;")
add_test([=[test_planners]=] "/Users/Dylan/Documents/autonomy_planning/gridplan/build/cp313-cp313-macosx_26_0_arm64/test_planners")
set_tests_properties([=[test_planners]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/Dylan/Documents/autonomy_planning/gridplan/CMakeLists.txt;59;add_test;/Users/Dylan/Documents/autonomy_planning/gridplan/CMakeLists.txt;0;")
subdirs("_deps/pybind11-build")
subdirs("_deps/googletest-build")
subdirs("_deps/googlebenchmark-build")
