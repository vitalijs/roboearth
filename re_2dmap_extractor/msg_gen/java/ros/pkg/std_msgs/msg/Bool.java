/* Auto-generated by genmsg_java.py for file /opt/ros/groovy/share/std_msgs/msg/Bool.msg */

package ros.pkg.std_msgs.msg;

import java.nio.ByteBuffer;

public class Bool extends ros.communication.Message {

  public boolean data;

  public Bool() {
  }

  public static java.lang.String __s_getDataType() { return "std_msgs/Bool"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "8b94c1b53db61fb6aed406028ad6332a"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "bool data\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public Bool clone() {
    Bool c = new Bool();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 1; // data
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.put((byte)(data ? 1 : 0));
  }

  public void deserialize(ByteBuffer bb) {
    data = bb.get() != 0 ? true : false;
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Bool))
      return false;
    Bool other = (Bool) o;
    return
      data == other.data &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.data ? 1231 : 1237);
    return result;
  }
} // class Bool

