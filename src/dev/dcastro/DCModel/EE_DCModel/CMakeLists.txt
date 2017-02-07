link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppDCModel_EE
    DCModel.cpp
    DCController.cpp
    AppDCModel.cpp
) 

