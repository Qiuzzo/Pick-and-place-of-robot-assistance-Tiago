#include <group_04_a2/human_client.h>

/// @brief Get the order of the objects from human_node
/// @return std::vector<int> The order of the objects
std::vector<int> get_order()
    {
        // use a service to get the objects
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
        tiago_iaslab_simulation::Objs srv;
        srv.request.ready = true;
        srv.request.all_objs = true;
        // wait two seconds for the service to become available
        ros::Duration(2).sleep();
        ROS_INFO("Requesting objects");
        if (client.call(srv))
        {
            ROS_INFO("Objects received");
        }
        else
        {
            ROS_ERROR("Failed to call service");
            ros::shutdown();
        }

        // Get the objects
        std::vector<int> objects = srv.response.ids;
        return objects;
}