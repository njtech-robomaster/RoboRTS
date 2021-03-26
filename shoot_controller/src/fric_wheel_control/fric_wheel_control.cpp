#include <roborts_msgs/FricWhl.h>
#include <roborts_msgs/GameStatus.h>
#include <ros/ros.h>

class FricWheelControl {
  public:
	FricWheelControl() {
		ros::NodeHandle nh;

		cmd_fric_wheel =
		    nh.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");

		game_status_sub = nh.subscribe<roborts_msgs::GameStatus>(
		    "game_status", 1,
		    [this](const roborts_msgs::GameStatus::ConstPtr &msg) {
			    bool fric_ready =
			        msg->game_status == roborts_msgs::GameStatus::FIVE_SEC_CD ||
			        msg->game_status == roborts_msgs::GameStatus::GAME;

			    if (!fric_status && fric_ready) {
				    fric_status = true;
				    roborts_msgs::FricWhl srv;
				    srv.request.open = true;
				    cmd_fric_wheel.call(srv);
				    ROS_INFO("Fric wheel on");

			    } else if (fric_status && !fric_ready) {
				    fric_status = false;
				    roborts_msgs::FricWhl srv;
				    srv.request.open = false;
				    cmd_fric_wheel.call(srv);
				    ROS_INFO("Fric wheel off");
			    }
		    });
	}

  private:
	ros::Subscriber game_status_sub;
	ros::ServiceClient cmd_fric_wheel;
	bool fric_status = false;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "fric_wheel_control");
	FricWheelControl ctrl;
	ros::spin();
}
