package moveit_msgs;

public interface PlaceAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "moveit_msgs/PlaceAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nmoveit_msgs/PlaceActionGoal action_goal\nmoveit_msgs/PlaceActionResult action_result\nmoveit_msgs/PlaceActionFeedback action_feedback\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  moveit_msgs.PlaceActionGoal getActionGoal();
  void setActionGoal(moveit_msgs.PlaceActionGoal value);
  moveit_msgs.PlaceActionResult getActionResult();
  void setActionResult(moveit_msgs.PlaceActionResult value);
  moveit_msgs.PlaceActionFeedback getActionFeedback();
  void setActionFeedback(moveit_msgs.PlaceActionFeedback value);
}