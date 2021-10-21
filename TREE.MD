~~~
├─scale_truck_control
│   ├── config
│   │   ├── config.yaml
│   │   ├── FV1.yaml
│   │   ├── FV2.yaml
│   │   ├── laser_filter.yaml
│   │   └── LV.yaml
│   ├── etc
│   │   ├── Controller
│   │   │   ├── build-Controller-Desktop-Debug
│   │   │   │   ├── Controller
│   │   │   │   ├── controller.o
│   │   │   │   ├── main.o
│   │   │   │   ├── Makefile
│   │   │   │   ├── moc_controller.cpp
│   │   │   │   ├── moc_controller.o
│   │   │   │   ├── moc_predefs.h
│   │   │   │   ├── moc_qTh.cpp
│   │   │   │   ├── moc_qTh.o
│   │   │   │   ├── qTh.o
│   │   │   │   ├── sock_udp.o
│   │   │   │   └── ui_controller.h
│   │   │   └── Controller
│   │   │       ├── controller.cpp
│   │   │       ├── controller.h
│   │   │       ├── Controller_ko_KR.ts
│   │   │       ├── Controller.pro
│   │   │       ├── Controller.pro.user
│   │   │       ├── controller.ui
│   │   │       ├── main.cpp
│   │   │       ├── qTh.cpp
│   │   │       ├── qTh.h
│   │   │       ├── sock_udp.cpp
│   │   │       └── sock_udp.hpp
│   │   ├── OpenCR
│   │   │   ├── FV1
│   │   │   │   ├── FV1.ino
│   │   │   │   └── FV1_LRT.ino
│   │   │   ├── FV2
│   │   │   │   └── FV2.ino
│   │   │   ├── LV
│   │   │   │   └── LV.ino
│   │   │   └── scale_truck_control_msgs.zip
│   │   └── Track
│   │       ├── drawing2.SLDDRW
│   │       ├── drawing_detail.pdf
│   │       ├── drawing_detail.SLDDRW
│   │       ├── drawing_detail.SLDPRT
│   │       ├── drawing.pdf
│   │       ├── drawing.png
│   │       ├── drawing.SLDDRW
│   │       ├── drawing.SLDPRT
│   │       ├── marking.pdf
│   │       ├── marking.SLDPRT
│   │       └── Virtual_Track_1.0.png
│   ├── include
│   │   ├── lane_detect
│   │   │   └── lane_detect.hpp
│   │   ├── object_detect
│   │   │   └── object_detect.hpp
│   │   ├── scale_truck_control
│   │   │   └── ScaleTruckController.hpp
│   │   └── sock_udp
│   │       └── sock_udp.hpp
│   ├── launch
│   │   ├── FV1.launch
│   │   ├── FV2.launch
│   │   ├── jpeg.launch
│   │   ├── laneCam_FV1.launch
│   │   ├── laneCam_FV2.launch
│   │   ├── lane_cam.launch
│   │   ├── laneCam_LV.launch
│   │   └── LV.launch
│   ├── msg
│   │   ├── ctl.msg
│   │   ├── lane_coef.msg
│   │   ├── lane.msg
│   │   └── vel.msg
│   ├── nodes
│   │   └── control_node.cpp
│   └── src
│       ├── lane_detect.cpp
│       ├── object_detect.cpp
│       ├── ScaleTruckController.cpp
│       └── sock_udp.cpp
├── CMakeLists.txt
├── README.md
├── TREE.md
└── package.xml
~~~
