#include "goal_executor.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

GoalExecutor::GoalExecutor()
    : global_planner{"global_planner_node_action"},
      local_planner{"local_planner_node_action"} {
	global_planner.waitForServer();
	local_planner.waitForServer();
}

void GoalExecutor::goto_goal(geometry_msgs::PoseStamped target,
                             std::function<void(bool)> callback) {
	roborts_msgs::GlobalPlannerGoal global_goal;
	global_goal.goal = target;

	global_planner.sendGoal(
	    global_goal,

	    [callback](const actionlib::SimpleClientGoalState &state,
	               const roborts_msgs::GlobalPlannerResultConstPtr &) {
		    callback(state == actionlib::SimpleClientGoalState::SUCCEEDED);
	    },

	    actionlib::SimpleActionClient<
	        roborts_msgs::GlobalPlannerAction>::SimpleActiveCallback(),

	    [this](const roborts_msgs::GlobalPlannerFeedbackConstPtr
	               &global_planner_feedback) {
		    if (!global_planner_feedback->path.poses.empty()) {
			    roborts_msgs::LocalPlannerGoal local_goal;
			    local_goal.route = global_planner_feedback->path;
			    local_planner.sendGoal(local_goal);
		    }
	    });
}

void GoalExecutor::goto_goal(geometry_msgs::Pose target,
                             std::function<void(bool)> callback) {
	geometry_msgs::PoseStamped stamped;
	stamped.pose = target;
	stamped.header.frame_id = "map";
	stamped.header.stamp = ros::Time::now();
	goto_goal(stamped, callback);
}

void GoalExecutor::goto_goal(geometry_msgs::Point target,
                             std::function<void(bool)> callback) {
	geometry_msgs::Pose pose1;
	pose1.position = target;
	tf2::Quaternion q1;
	q1.setRPY(0, 0, 0);
	pose1.orientation = tf2::toMsg(q1);

	goto_goal(pose1, [this, target, callback](bool) {
		geometry_msgs::Pose pose2;
		pose2.position = target;
		tf2::Quaternion q2;
		q2.setRPY(0, 0, M_PI);
		pose2.orientation = tf2::toMsg(q2);

		goto_goal(pose2, callback);
	});
}
