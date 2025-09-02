#include <ros/ros.h>
#include <rospy_tutorials/AddTwoInts.h>

bool handle_add_two_ints(rospy_tutorials::AddTwoInts::Request &req, rospy_tutorials::AddTwoInts::Response &res)
{
    int result = req.a + req.b;
    ROS_INFO("%d + %d = %d", (int)req.a, (int)req.b, (int)result);
    res.sum = result;
    return true;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<rospy_tutorials::AddTwoInts>("/add_two_ints");

    rospy_tutorials::AddTwoInts srv;
    srv.request.a = 5;
    srv.request.b = 6;

    if (client.call(srv)) {
        ROS_INFO("Returned sum is %d", (int)srv.response.sum);
    } else {
	ROS_WARN("Service call failed");
    }
}
