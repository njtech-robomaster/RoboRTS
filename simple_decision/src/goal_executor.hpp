#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <roborts_msgs/GlobalPlannerAction.h>
#include <roborts_msgs/LocalPlannerAction.h>

class GoalExecutor {
  public:
	GoalExecutor();

	void goto_goal(geometry_msgs::PoseStamped target,
	               std::function<void(bool)> callback);

	void goto_goal(geometry_msgs::Pose target,
	               std::function<void(bool)> callback);

	void goto_goal(geometry_msgs::Point target,
	               std::function<void(bool)> callback);

  private:
	actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction>
	    global_planner;
	actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction>
	    local_planner;
};
