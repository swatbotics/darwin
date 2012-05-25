if (WIN32)

  set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH};C:\\OpenCV2.2\\lib)
  include_directories(C:\\OpenCV2.2\\include C:\\OpenCV2.2\\include\\opencv)
  add_definitions(-D_USE_MATH_DEFINES)

  find_library(OPENCV_CALIB3D_LIBRARY opencv_calib3d220)
  find_library(OPENCV_CONTRIB_LIBRARY opencv_contrib220)
  find_library(OPENCV_CORE_LIBRARY opencv_core220)
  find_library(OPENCV_FEATURES2D_LIBRARY opencv_features2d220)
  find_library(OPENCV_FFMPEG_LIBRARY opencv_ffmpeg220)
  find_library(OPENCV_FLANN_LIBRARY opencv_flann220)
  find_library(OPENCV_GPU_LIBRARY opencv_gpu220)
  find_library(OPENCV_HIGHGUI_LIBRARY opencv_highgui220)
  find_library(OPENCV_IMGPROC_LIBRARY opencv_imgproc220)
  find_library(OPENCV_LEGACY_LIBRARY opencv_legacy220)
  find_library(OPENCV_ML_LIBRARY opencv_ml220)
  find_library(OPENCV_OBJDETECT_LIBRARY opencv_objdetect220)
  find_library(OPENCV_TS_LIBRARY opencv_ts220)
  find_library(OPENCV_VIDEO_LIBRARY opencv_video220)

  find_library(OPENCV_CALIB3D_DEBUG_LIBRARY opencv_calib3d220d)
  find_library(OPENCV_CONTRIB_DEBUG_LIBRARY opencv_contrib220d)
  find_library(OPENCV_CORE_DEBUG_LIBRARY opencv_core220d)
  find_library(OPENCV_FEATURES2D_DEBUG_LIBRARY opencv_features2d220d)
  find_library(OPENCV_FFMPEG_DEBUG_LIBRARY opencv_ffmpeg220d)
  find_library(OPENCV_FLANN_DEBUG_LIBRARY opencv_flann220d)
  find_library(OPENCV_GPU_DEBUG_LIBRARY opencv_gpu220d)
  find_library(OPENCV_HIGHGUI_DEBUG_LIBRARY opencv_highgui220d)
  find_library(OPENCV_IMGPROC_DEBUG_LIBRARY opencv_imgproc220d)
  find_library(OPENCV_LEGACY_DEBUG_LIBRARY opencv_legacy220d)
  find_library(OPENCV_ML_DEBUG_LIBRARY opencv_ml220d)
  find_library(OPENCV_OBJDETECT_DEBUG_LIBRARY opencv_objdetect220d)
  find_library(OPENCV_TS_DEBUG_LIBRARY opencv_ts220d)
  find_library(OPENCV_VIDEO_DEBUG_LIBRARY opencv_video220d)

  set(OPENCV_LIBRARIES 
    ${OPENCV_CALIB3D_LIBRARY}
    ${OPENCV_CONTRIB_LIBRARY}
    ${OPENCV_CORE_LIBRARY}
    ${OPENCV_FEATURES2D_LIBRARY}
    ${OPENCV_FFMPEG_LIBRARY}
    ${OPENCV_FLANN_LIBRARY}
    ${OPENCV_GPU_LIBRARY}
    ${OPENCV_HIGHGUI_LIBRARY}
    ${OPENCV_IMGPROC_LIBRARY}
    ${OPENCV_LEGACY_LIBRARY}
    ${OPENCV_ML_LIBRARY}
    ${OPENCV_OBJDETECT_LIBRARY}
    ${OPENCV_TS_LIBRARY}
    ${OPENCV_VIDEO_LIBRARY})


  set(OPENCV_DEBUG_LIBRARIES 
    ${OPENCV_CALIB3D_DEBUG_LIBRARY}
    ${OPENCV_CONTRIB_DEBUG_LIBRARY}
    ${OPENCV_CORE_DEBUG_LIBRARY}
    ${OPENCV_FEATURES2D_DEBUG_LIBRARY}
    ${OPENCV_FFMPEG_DEBUG_LIBRARY}
    ${OPENCV_FLANN_DEBUG_LIBRARY}
    ${OPENCV_GPU_DEBUG_LIBRARY}
    ${OPENCV_HIGHGUI_DEBUG_LIBRARY}
    ${OPENCV_IMGPROC_DEBUG_LIBRARY}
    ${OPENCV_LEGACY_DEBUG_LIBRARY}
    ${OPENCV_ML_DEBUG_LIBRARY}
    ${OPENCV_OBJDETECT_DEBUG_LIBRARY}
    ${OPENCV_VIDEO_DEBUG_LIBRARY})


else() # Mac OS X / Linux

  if(APPLE)
    set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/local/lib)
    include_directories(/opt/local/include)
    include_directories(/opt/local/include/opencv)
  else() # Linux
    set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /usr/local/lib)
    include_directories(/usr/local/include)
    include_directories(/usr/local/include/opencv)
  endif()

  include_directories(/usr/include/opencv-2.3.1)

  find_library(OPENCV_CORE_LIBRARY opencv_core)
  find_library(OPENCV_IMGPROC_LIBRARY opencv_imgproc)
  find_library(OPENCV_HIGHGUI_LIBRARY opencv_highgui)
  find_library(OPENCV_ML_LIBRARY opencv_ml)
  find_library(OPENCV_FEATURES2D_LIBRARY opencv_features2d)
  find_library(OPENCV_VIDEO_LIBRARY opencv_video)
  find_library(OPENCV_OBJDETECT_LIBRARY opencv_objdetect)
  find_library(OPENCV_CALIB3D_LIBRARY opencv_calib3d)
  find_library(OPENCV_FLANN_LIBRARY opencv_flann)

  set(OPENCV_LIBRARIES 
    ${OPENCV_CORE_LIBRARY}
    ${OPENCV_IMGPROC_LIBRARY}
    ${OPENCV_HIGHGUI_LIBRARY}
    ${OPENCV_ML_LIBRARY}
    ${OPENCV_FEATURES2D_LIBRARY}
    ${OPENCV_VIDEO_LIBRARY}
    ${OPENCV_OBJDETECT_LIBRARY}
    ${OPENCV_CALIB3D_LIBRARY}
    ${OPENCV_FLANN_LIBRARY})

  set(OPENCV_DEBUG_LIBRARIES ${OPENCV_LIBRARIES})

endif()

function(add_opencv_libs target)

  foreach(lib ${OPENCV_LIBRARIES})
    target_link_libraries(${target} optimized ${lib})
  endforeach(lib)

  foreach(lib ${OPENCV_DEBUG_LIBRARIES})
    target_link_libraries(${target} debug ${lib})
  endforeach(lib)

endfunction(add_opencv_libs)
