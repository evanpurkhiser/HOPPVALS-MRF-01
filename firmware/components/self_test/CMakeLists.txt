idf_component_register(
    SRCS         "src/self_test.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     motor encoder current_sense motion types
)
