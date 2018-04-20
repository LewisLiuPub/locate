indoor localization node;


main function;
1) detect localization failure:
mainly calculate match error between scan and map;


2) record successful match result, use this result to update odom;
ensure odom always get correct and stable location;

3) take charge of updating odometry