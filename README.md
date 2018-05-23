indoor localization node;


main function;
1) detect localization failure:
mainly calculate match error between scan and map;


2) record successful match result, use this result to update odom;
ensure odom always get correct and stable location;

3) take charge of updating odometry

----
- buid amcl bug:
    catkin_ws/build/localization/amcl/setup_custom_pythonpath.sh: 4: exec: /home/muyi/catkin_ws/src/localization/amcl/cfg/AMCL.cfg: Permission denied
    make[2]: *** [catkin_ws/devel/include/amcl/AMCLConfig.h] Error 126
    make[1]: *** [localization/amcl/CMakeFiles/amcl_gencfg.dir/all] Error 2
    make: *** [all] Error 2
    Invoking "make -j4 -l4" failed
    
    
    sudo chmod 755 src/localization/amcl/cfg/AMCL.cfg 

- matrix lib
    https://github.com/AmberThrall/TinyMatrix.git
