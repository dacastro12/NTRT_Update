link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppDC1_2Model
    DC1_2Model.cpp
    DC1_2Controller.cpp
    AppDC1_2Model.cpp
) 

