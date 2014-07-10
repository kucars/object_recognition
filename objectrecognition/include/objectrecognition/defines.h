// DEFINES FILE
#ifndef DEFINES
#define DEFINES

const double EPSILON = 0.0001;
const double PI = 3.14159265;
const double ANGLE_MAX = (2*PI);
const double INTERNAL_PRODUCT_INTERVAL = 2.0000; // [-1,1] 180 degrees
const double RAD_TO_DEG = (180/PI);

const double NORMAL_DISPLAY_DISPARITY = 10;
const double NORMAL_SCALE = 0.01;
const double RADIUS_SEARCH = 0.03;


const std::string GET_MODEL_LIST_SRV = "/objects_database_node/get_model_list";
const std::string GET_MODEL_MESH_SRV = "/objects_database_node/get_model_mesh";

#endif //#ifndef DEFINES
