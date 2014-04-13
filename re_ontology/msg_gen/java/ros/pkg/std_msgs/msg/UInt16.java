/* Auto-generated by genmsg_java.py for file /opt/ros/groovy/share/std_msgs/msg/UInt16.msg */

package ros.pkg.std_msgs.msg;

import java.nio.ByteBuffer;

public class UInt16 extends ros.communication.Message {

  public int data;

  public UInt16() {
  }

  public static java.lang.String __s_getDataType() { return "std_msgs/UInt16"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "1df79edf208b629fe6b81923a544552d"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "uint16 data\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public UInt16 clone() {
    UInt16 c = new UInt16();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 2; // data
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.putShort((short)data);
  }

  public void deserialize(ByteBuffer bb) {
    data = (int)(bb.getShort() & 0xffff);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof UInt16))
      return false;
    UInt16 other = (UInt16) o;
    return
      data == other.data &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + this.data;
    return result;
  }
} // class UInt16
