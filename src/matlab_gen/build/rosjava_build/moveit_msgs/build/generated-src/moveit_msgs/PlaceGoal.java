package moveit_msgs;

public interface PlaceGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "moveit_msgs/PlaceGoal";
  static final java.lang.String _DEFINITION = "# An action for placing an object\n\n# which group to be used to plan for grasping\nstring group_name\n\n# the name that the attached object to place\nstring attached_object_name\n\n# a list of possible transformations for placing the object\nPlaceLocation[] place_locations\n\n# if the user prefers setting the eef pose (same as in pick) rather than \n# the location of an end effector, this flag should be set to true\nbool place_eef\n\n# the name that the support surface (e.g. table) has in the collision world\n# can be left empty if no name is available\nstring support_surface_name\n\n# whether collisions between the gripper and the support surface should be acceptable\n# during move from pre-place to place and during retreat. Collisions when moving to the\n# pre-place location are still not allowed even if this is set to true.\nbool allow_gripper_support_collision\n\n# Optional constraints to be imposed on every point in the motion plan\nConstraints path_constraints\n\n# The name of the motion planner to use. If no name is specified,\n# a default motion planner will be used\nstring planner_id\n\n# an optional list of obstacles that we have semantic information about\n# and that can be touched/pushed/moved in the course of grasping;\n# CAREFUL: If the object name \'all\' is used, collisions with all objects are disabled during the approach & lift.\nstring[] allowed_touch_objects\n\n# The maximum amount of time the motion planner is allowed to plan for\nfloat64 allowed_planning_time\n\n# Planning options\nPlanningOptions planning_options\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  java.lang.String getGroupName();
  void setGroupName(java.lang.String value);
  java.lang.String getAttachedObjectName();
  void setAttachedObjectName(java.lang.String value);
  java.util.List<moveit_msgs.PlaceLocation> getPlaceLocations();
  void setPlaceLocations(java.util.List<moveit_msgs.PlaceLocation> value);
  boolean getPlaceEef();
  void setPlaceEef(boolean value);
  java.lang.String getSupportSurfaceName();
  void setSupportSurfaceName(java.lang.String value);
  boolean getAllowGripperSupportCollision();
  void setAllowGripperSupportCollision(boolean value);
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
