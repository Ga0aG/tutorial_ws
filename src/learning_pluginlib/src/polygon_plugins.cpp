#include <pluginlib/class_list_macros.h>//Here, we include the pluginlib macros that allow us to register classes as plugins.
#include <learning_pluginlib/polygon_base.h>
#include <learning_pluginlib/polygon_plugins.h>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)

//The steps above make it so that instances of our plugins can be created 
//once the library they exist in is loaded, 
//but the plugin loader still needs a way to find that library 
//and to know what to reference within that library. 
