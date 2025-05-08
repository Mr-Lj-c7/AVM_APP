add_library(2d_surround SHARED 
    src/ParamSettings.cpp
    src/FisheysCameraModel.cpp
    src/CalibrateCamera.cpp
    src/BirdView.cpp
    src/UtilsView.cpp
)

target_link_libraries(2d_surround ${OpenCV_LIBS})
