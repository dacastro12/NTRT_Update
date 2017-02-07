link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

subdirs(
    EE_DCModel
)

add_executable(AppDCModel
    DCModel.cpp
    DCController.cpp
    AppDCModel.cpp
    ) 

