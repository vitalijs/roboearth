/* Auto-generated by genmsg_java.py for file /opt/ros/groovy/stacks/arm_navigation/arm_navigation_msgs/msg/MoveArmStatistics.msg */

package ros.pkg.arm_navigation_msgs.msg;

import java.nio.ByteBuffer;

public class MoveArmStatistics extends ros.communication.Message {

  public int request_id;
  public java.lang.String result = new java.lang.String();
  public ros.pkg.arm_navigation_msgs.msg.ArmNavigationErrorCodes error_code = new ros.pkg.arm_navigation_msgs.msg.ArmNavigationErrorCodes();
  public double planning_time;
  public double smoothing_time;
  public double ik_time;
  public double time_to_execution;
  public double time_to_result;
  public boolean preempted;
  public double num_replans;
  public double trajectory_duration;
  public java.lang.String planner_service_name = new java.lang.String();

  public MoveArmStatistics() {
  }

  public static java.lang.String __s_getDataType() { return "arm_navigation_msgs/MoveArmStatistics"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "d83dee1348791a0d1414257b41bc161f"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "int32 request_id\n" +
"string result\n" +
"arm_navigation_msgs/ArmNavigationErrorCodes error_code\n" +
"\n" +
"float64 planning_time\n" +
"float64 smoothing_time\n" +
"float64 ik_time\n" +
"float64 time_to_execution\n" +
"float64 time_to_result\n" +
"\n" +
"bool preempted\n" +
"\n" +
"float64 num_replans\n" +
"float64 trajectory_duration\n" +
"\n" +
"string planner_service_name\n" +
"\n" +
"================================================================================\n" +
"MSG: arm_navigation_msgs/ArmNavigationErrorCodes\n" +
"int32 val\n" +
"\n" +
"# overall behavior\n" +
"int32 PLANNING_FAILED=-1\n" +
"int32 SUCCESS=1\n" +
"int32 TIMED_OUT=-2\n" +
"\n" +
"# start state errors\n" +
"int32 START_STATE_IN_COLLISION=-3\n" +
"int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-4\n" +
"\n" +
"# goal errors\n" +
"int32 GOAL_IN_COLLISION=-5\n" +
"int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-6\n" +
"\n" +
"# robot state\n" +
"int32 INVALID_ROBOT_STATE=-7\n" +
"int32 INCOMPLETE_ROBOT_STATE=-8\n" +
"\n" +
"# planning request errors\n" +
"int32 INVALID_PLANNER_ID=-9\n" +
"int32 INVALID_NUM_PLANNING_ATTEMPTS=-10\n" +
"int32 INVALID_ALLOWED_PLANNING_TIME=-11\n" +
"int32 INVALID_GROUP_NAME=-12\n" +
"int32 INVALID_GOAL_JOINT_CONSTRAINTS=-13\n" +
"int32 INVALID_GOAL_POSITION_CONSTRAINTS=-14\n" +
"int32 INVALID_GOAL_ORIENTATION_CONSTRAINTS=-15\n" +
"int32 INVALID_PATH_JOINT_CONSTRAINTS=-16\n" +
"int32 INVALID_PATH_POSITION_CONSTRAINTS=-17\n" +
"int32 INVALID_PATH_ORIENTATION_CONSTRAINTS=-18\n" +
"\n" +
"# state/trajectory monitor errors\n" +
"int32 INVALID_TRAJECTORY=-19\n" +
"int32 INVALID_INDEX=-20\n" +
"int32 JOINT_LIMITS_VIOLATED=-21\n" +
"int32 PATH_CONSTRAINTS_VIOLATED=-22\n" +
"int32 COLLISION_CONSTRAINTS_VIOLATED=-23\n" +
"int32 GOAL_CONSTRAINTS_VIOLATED=-24\n" +
"int32 JOINTS_NOT_MOVING=-25\n" +
"int32 TRAJECTORY_CONTROLLER_FAILED=-26\n" +
"\n" +
"# system errors\n" +
"int32 FRAME_TRANSFORM_FAILURE=-27\n" +
"int32 COLLISION_CHECKING_UNAVAILABLE=-28\n" +
"int32 ROBOT_STATE_STALE=-29\n" +
"int32 SENSOR_INFO_STALE=-30\n" +
"\n" +
"# kinematics errors\n" +
"int32 NO_IK_SOLUTION=-31\n" +
"int32 INVALID_LINK_NAME=-32\n" +
"int32 IK_LINK_IN_COLLISION=-33\n" +
"int32 NO_FK_SOLUTION=-34\n" +
"int32 KINEMATICS_STATE_IN_COLLISION=-35\n" +
"\n" +
"# general errors\n" +
"int32 INVALID_TIMEOUT=-36\n" +
"\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public MoveArmStatistics clone() {
    MoveArmStatistics c = new MoveArmStatistics();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 4; // request_id
    __l += 4 + result.length();
    __l += error_code.serializationLength();
    __l += 8; // planning_time
    __l += 8; // smoothing_time
    __l += 8; // ik_time
    __l += 8; // time_to_execution
    __l += 8; // time_to_result
    __l += 1; // preempted
    __l += 8; // num_replans
    __l += 8; // trajectory_duration
    __l += 4 + planner_service_name.length();
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.putInt(request_id);
    Serialization.writeString(bb, result);
    error_code.serialize(bb, seq);
    bb.putDouble(planning_time);
    bb.putDouble(smoothing_time);
    bb.putDouble(ik_time);
    bb.putDouble(time_to_execution);
    bb.putDouble(time_to_result);
    bb.put((byte)(preempted ? 1 : 0));
    bb.putDouble(num_replans);
    bb.putDouble(trajectory_duration);
    Serialization.writeString(bb, planner_service_name);
  }

  public void deserialize(ByteBuffer bb) {
    request_id = bb.getInt();
    result = Serialization.readString(bb);
    error_code.deserialize(bb);
    planning_time = bb.getDouble();
    smoothing_time = bb.getDouble();
    ik_time = bb.getDouble();
    time_to_execution = bb.getDouble();
    time_to_result = bb.getDouble();
    preempted = bb.get() != 0 ? true : false;
    num_replans = bb.getDouble();
    trajectory_duration = bb.getDouble();
    planner_service_name = Serialization.readString(bb);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof MoveArmStatistics))
      return false;
    MoveArmStatistics other = (MoveArmStatistics) o;
    return
      request_id == other.request_id &&
      result.equals(other.result) &&
      error_code.equals(other.error_code) &&
      planning_time == other.planning_time &&
      smoothing_time == other.smoothing_time &&
      ik_time == other.ik_time &&
      time_to_execution == other.time_to_execution &&
      time_to_result == other.time_to_result &&
      preempted == other.preempted &&
      num_replans == other.num_replans &&
      trajectory_duration == other.trajectory_duration &&
      planner_service_name.equals(other.planner_service_name) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + this.request_id;
    result = prime * result + (this.result == null ? 0 : this.result.hashCode());
    result = prime * result + (this.error_code == null ? 0 : this.error_code.hashCode());
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.planning_time)) ^ (tmp >>> 32));
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.smoothing_time)) ^ (tmp >>> 32));
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.ik_time)) ^ (tmp >>> 32));
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.time_to_execution)) ^ (tmp >>> 32));
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.time_to_result)) ^ (tmp >>> 32));
    result = prime * result + (this.preempted ? 1231 : 1237);
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.num_replans)) ^ (tmp >>> 32));
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.trajectory_duration)) ^ (tmp >>> 32));
    result = prime * result + (this.planner_service_name == null ? 0 : this.planner_service_name.hashCode());
    return result;
  }
} // class MoveArmStatistics

