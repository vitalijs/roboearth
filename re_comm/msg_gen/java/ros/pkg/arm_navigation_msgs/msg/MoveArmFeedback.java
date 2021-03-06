/* Auto-generated by genmsg_java.py for file /opt/ros/groovy/stacks/arm_navigation/arm_navigation_msgs/msg/MoveArmFeedback.msg */

package ros.pkg.arm_navigation_msgs.msg;

import java.nio.ByteBuffer;

public class MoveArmFeedback extends ros.communication.Message {

  public java.lang.String state = new java.lang.String();
  public ros.communication.Duration time_to_completion = new ros.communication.Duration();

  public MoveArmFeedback() {
  }

  public static java.lang.String __s_getDataType() { return "arm_navigation_msgs/MoveArmFeedback"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "321f3feadd0d5c1b7d7135738e673560"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n" +
"# The internal state that the move arm action currently is in\n" +
"string state\n" +
"\n" +
"# Time to completion - this is a combination of requested planning time and trajectory completion time\n" +
"duration time_to_completion\n" +
"\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public MoveArmFeedback clone() {
    MoveArmFeedback c = new MoveArmFeedback();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 4 + state.length();
    __l += 8; // time_to_completion
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    Serialization.writeString(bb, state);
    Serialization.writeDuration(bb, time_to_completion);
  }

  public void deserialize(ByteBuffer bb) {
    state = Serialization.readString(bb);
    time_to_completion = Serialization.readDuration(bb);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof MoveArmFeedback))
      return false;
    MoveArmFeedback other = (MoveArmFeedback) o;
    return
      state.equals(other.state) &&
      time_to_completion.equals(other.time_to_completion) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.state == null ? 0 : this.state.hashCode());
    result = prime * result + (this.time_to_completion == null ? 0 : this.time_to_completion.hashCode());
    return result;
  }
} // class MoveArmFeedback

