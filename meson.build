project('vision', 'cpp', default_options : ['cpp_std=c++20'])

cc = meson.get_compiler('cpp')

# Opencv
opencv_dirs = ['/usr/lib/aarch64-linux-gnu']
opencv_deps = [
               cc.find_library('opencv_core', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_calib3d', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_videoio', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_rgbd', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_video', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_imgproc', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_plot', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_highgui', required: true, dirs: opencv_dirs),
               cc.find_library('opencv_imgcodecs', required: true, dirs: opencv_dirs),
              ]
opencv_inc = include_directories('/usr/include/opencv4')

# Wpi
wpi_deps = [
            cc.find_library('wpinet', required: true),
            cc.find_library('ntcore', required: true),
           ]

wpi_incs = [
            include_directories('/usr/local/include/ntcore/'),
            include_directories('/usr/local/include/wpiutil/'),
            include_directories('/usr/local/include/wpinet/'),
           ]

spin_deps = [
             cc.find_library('Spinnaker', dirs: '/opt/spinnaker/lib', required: true),
            ]

spin_incs = [
             include_directories('/opt/spinnaker/include/'),
            ]

stream_incs = [
               include_directories('./cpp-mjpeg-streamer/single_include/')
              ]

# THE REAL STUFF

src = [
       'main.cpp',
       'pose_estimation.cpp',
       'detection.cpp',
       'Cameras.cpp',
       'flirCamera.cpp',
       'depthCamera.cpp',
       'usbCamera.cpp',
       'graphics_helpers.cpp',
      ]

deps = [
        opencv_deps,
        dependency('realsense2'),
        dependency('eigen3'),
        dependency('apriltag'),
        dependency('Threads'),
        wpi_deps,
        spin_deps,
       ]

inc = [
       opencv_inc,
       wpi_incs,
       spin_incs,
       stream_incs,
      ]

executable('runvision',
           sources : src,
           dependencies : deps,
           include_directories : inc,
           cpp_args : ['-Wno-deprecated', '-Wno-non-virtual-dtor', '-Wno-psabi'])
