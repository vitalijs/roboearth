/* Auto-generated by genmsg_cpp for file /home/vitalij/ros/stacks/roboearth/re_srvs/srv/Request2DMap.srv */

package ros.pkg.re_srvs.srv;

import java.nio.ByteBuffer;


public class Request2DMap extends ros.communication.Service<Request2DMap.Request, Request2DMap.Response> {

  public static java.lang.String __s_getDataType() { return "re_srvs/Request2DMap"; }
  public static java.lang.String __s_getMD5Sum() { return "36589988f0eaacd3eb9e9e443f14ac19"; }

  public java.lang.String getDataType() { return Request2DMap.__s_getDataType(); }
  public java.lang.String getMD5Sum() { return Request2DMap.__s_getMD5Sum(); }

  public Request2DMap.Request createRequest() {
    return new Request2DMap.Request();
  }

  public Request2DMap.Response createResponse() {
    return new Request2DMap.Response();
  }

static public class Request extends ros.communication.Message {

  public java.lang.String envUID = new java.lang.String();
  public java.lang.String srdl = new java.lang.String();
  public java.lang.String baseScannerLink = new java.lang.String();
  public java.lang.String targetMapName = new java.lang.String();

  public Request() {
  }

  public static java.lang.String __s_getDataType() { return "re_srvs/Request2DMapRequest"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "4ecd72f87dfe5c9d4a80ccd12672f9b3"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getServerMD5Sum() { return "36589988f0eaacd3eb9e9e443f14ac19"; }
  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "string envUID\n" +
"string srdl\n" +
"string baseScannerLink\n" +
"\n" +
"string targetMapName\n" +
"\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public Request clone() {
    Request c = new Request();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 4 + envUID.length();
    __l += 4 + srdl.length();
    __l += 4 + baseScannerLink.length();
    __l += 4 + targetMapName.length();
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    Serialization.writeString(bb, envUID);
    Serialization.writeString(bb, srdl);
    Serialization.writeString(bb, baseScannerLink);
    Serialization.writeString(bb, targetMapName);
  }

  public void deserialize(ByteBuffer bb) {
    envUID = Serialization.readString(bb);
    srdl = Serialization.readString(bb);
    baseScannerLink = Serialization.readString(bb);
    targetMapName = Serialization.readString(bb);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Request))
      return false;
    Request other = (Request) o;
    return
      envUID.equals(other.envUID) &&
      srdl.equals(other.srdl) &&
      baseScannerLink.equals(other.baseScannerLink) &&
      targetMapName.equals(other.targetMapName) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.envUID == null ? 0 : this.envUID.hashCode());
    result = prime * result + (this.srdl == null ? 0 : this.srdl.hashCode());
    result = prime * result + (this.baseScannerLink == null ? 0 : this.baseScannerLink.hashCode());
    result = prime * result + (this.targetMapName == null ? 0 : this.targetMapName.hashCode());
    return result;
  }
} // class Request

static public class Response extends ros.communication.Message {

  public boolean success;
  public ros.pkg.re_msgs.msg.RosFile map = new ros.pkg.re_msgs.msg.RosFile();
  public ros.pkg.re_msgs.msg.RosFile meta = new ros.pkg.re_msgs.msg.RosFile();

  public Response() {
  }

  public static java.lang.String __s_getDataType() { return "re_srvs/Request2DMapResponse"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "0b14b385997efc59caa091c489f65080"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getServerMD5Sum() { return "36589988f0eaacd3eb9e9e443f14ac19"; }
  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "bool success\n" +
"re_msgs/RosFile map\n" +
"re_msgs/RosFile meta\n" +
"\n" +
"\n" +
"================================================================================\n" +
"MSG: re_msgs/RosFile\n" +
"# This file representation is used to pass binary data to the RoboEarthDB.\n" +
"# As the endianess isn't stored, only files with a byte order mark (BOM) or\n" +
"# an implicitly specified endianess should be transferred.\n" +
"string name   # file name\n" +
"int8[] data   # binary data \n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public Response clone() {
    Response c = new Response();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 1; // success
    __l += map.serializationLength();
    __l += meta.serializationLength();
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.put((byte)(success ? 1 : 0));
    map.serialize(bb, seq);
    meta.serialize(bb, seq);
  }

  public void deserialize(ByteBuffer bb) {
    success = bb.get() != 0 ? true : false;
    map.deserialize(bb);
    meta.deserialize(bb);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Response))
      return false;
    Response other = (Response) o;
    return
      success == other.success &&
      map.equals(other.map) &&
      meta.equals(other.meta) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.success ? 1231 : 1237);
    result = prime * result + (this.map == null ? 0 : this.map.hashCode());
    result = prime * result + (this.meta == null ? 0 : this.meta.hashCode());
    return result;
  }
} // class Response

} //class

