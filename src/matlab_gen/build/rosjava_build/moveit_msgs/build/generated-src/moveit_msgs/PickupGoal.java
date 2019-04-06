package moveit_msgs;

public interface PickupGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "moveit_msgs/PickupGoal";
  static final java.lang.String _DEFINITION = "# An action for picking up an object\n\n# The name of the object to pick up (as known in the planning scene)\nstring target_name\n\n# which group should be used to plan for pickup\nstring group_name\n\n# which end-effector to be used for pickup (ideally descending from the group above)\nstring end_effector\n\n# a list of possible grasps to be used. At least one grasp must be filled in\nGrasp[] possible_grasps\n\n# the name that the support surface (e.g. table) has in the collision map\n# can be left empty if no name is available\nstring support_surface_name\n\n# whether collisions between the gripper and the support surface should be acceptable\n# during move from pre-grasp to grasp and during lift. Collisions when moving to the\n# pre-grasp location are still not allowed even if this is set to true.\nbool allow_gripper_support_collision\n\n# The names of the links the object to be attached is allowed to touch;\n# If this is left empty, it defaults to the links in the used end-effector\nstring[] attached_object_touch_links\n\n# Optionally notify the pick action that it should approach the object further,\n# as much as possible (this minimizing the distance to the object before the grasp)\n# along the approach direction; Note: this option changes the grasping poses\n# supplied in possible_grasps[] such that they are closer to the object when possible.\nbool minimize_object_distance\n\n# Optional constraints to be imposed on every point in the motion plan\nConstraints path_constraints\n\n# The name of the motion planner to use. If no name is specified,\n# a default motion planner will be used\nstring planner_id\n\n# an optional list of obstacles that we have semantic information about\n# and that can be touched/pushed/moved in the course of grasping;\n# CAREFUL: If the object name \'all\' is used, collisions with all objects are disabled during the approach & lift.\nstring[] allowed_touch_objects\n\n# The maximum amount of time the motion planner is allowed to plan for\nfloat64 allowed_planning_time\n\n# Planning options\nPlanningOptions planning_options\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  java.lang.String getTargetName();
  void setTargetName(java.lang.String value);
  java.lang.String getGroupName();
  void setGroupName(java.lang.String value);
  java.lang.String getEndEffector();
  void setEndEffector(java.lang.String value);
  java.util.List<moveit_msgs.Grasp> getPossibleGrasps();
  void setPossibleGrasps(java.util.List<moveit_msgs.Grasp> value);
  java.lang.String getSupportSurfaceName();
  void setSupportSurfaceName(java.lang.String value);
  boolean getAllowGripperSupportCollision();
  void setAllowGripperSupportCollision(boolean value);
  java.util.List<java.lang.String> getAttachedObjectTouchLinks();
  void setAttachedObjectTouchLinks(java.util.List<java.lang.String> value);
  boolean getMinimizeObjectDistance();
  void setMinimizeObjectDistance(boolean value);
  moveit_msgs.Constraints getPathConstraints();
  void setPathConstraints(moveit_msgs.Constraints value);
  java.lang.String getPlannerId();
  void setPlannerId(java.lang.String value);
  java.util.List<java.lang.String> getAllowedTouchObjects();
  void setAllowedTouchObjects(java.util.List<java.lang.String> value);
  double getAllowedPlanningTime();
  void setAllowedPlanningTime(double value);
  moveit_msgs.PlanningOptions getPlanningOptions();
  void setPlanningOptions(moveit_msgs.PlanningOptions value);
}
