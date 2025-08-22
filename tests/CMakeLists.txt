find_package(GTest REQUIRED)

add_executable(FECEncoderTests FECEncoderTests.cpp)
target_link_libraries(FECEncoderTests GTest::GTest GTest::Main milstd110c)
add_test(NAME FECEncoderTests COMMAND FECEncoderTests)

add_executable(InterleaverTests InterleaverTests.cpp)
target_link_libraries(InterleaverTests GTest::GTest GTest::Main milstd110c)
add_test(NAME InterleaverTests COMMAND InterleaverTests)
