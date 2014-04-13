/* Auto-generated by genmsg_cpp for file /home/vitalij/ros/stacks/roboearth/re_srvs/srv/SearchActionRecipes.srv */

package ros.pkg.re_srvs.srv;

import java.nio.ByteBuffer;


public class SearchActionRecipes extends ros.communication.Service<SearchActionRecipes.Request, SearchActionRecipes.Response> {

  public static java.lang.String __s_getDataType() { return "re_srvs/SearchActionRecipes"; }
  public static java.lang.String __s_getMD5Sum() { return "ab460e156aa3e532e70c8a5b8e2f72e7"; }

  public java.lang.String getDataType() { return SearchActionRecipes.__s_getDataType(); }
  public java.lang.String getMD5Sum() { return SearchActionRecipes.__s_getMD5Sum(); }

  public SearchActionRecipes.Request createRequest() {
    return new SearchActionRecipes.Request();
  }

  public SearchActionRecipes.Response createResponse() {
    return new SearchActionRecipes.Response();
  }

static public class Request extends ros.communication.Message {

  public java.lang.String searchID = new java.lang.String();

  public Request() {
  }

  public static java.lang.String __s_getDataType() { return "re_srvs/SearchActionRecipesRequest"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "2d2d663456e9fa76707235a598ce31c7"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getServerMD5Sum() { return "ab460e156aa3e532e70c8a5b8e2f72e7"; }
  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "string searchID\n" +
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
    __l += 4 + searchID.length();
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    Serialization.writeString(bb, searchID);
  }

  public void deserialize(ByteBuffer bb) {
    searchID = Serialization.readString(bb);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Request))
      return false;
    Request other = (Request) o;
    return
      searchID.equals(other.searchID) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.searchID == null ? 0 : this.searchID.hashCode());
    return result;
  }
} // class Request

static public class Response extends ros.communication.Message {

  public boolean success;
  public java.util.ArrayList<java.lang.String> uids = new java.util.ArrayList<java.lang.String>();
  public java.util.ArrayList<java.lang.String> recipes = new java.util.ArrayList<java.lang.String>();

  public Response() {
  }

  public static java.lang.String __s_getDataType() { return "re_srvs/SearchActionRecipesResponse"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "8dad7a939c1eb856368da22a41bc0845"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getServerMD5Sum() { return "ab460e156aa3e532e70c8a5b8e2f72e7"; }
  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "bool success\n" +
"string[] uids\n" +
"string[] recipes\n" +
"\n" +
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
    __l += 4;
    for(java.lang.String val : uids) {
      __l += 4 + val.length();
    }
    __l += 4;
    for(java.lang.String val : recipes) {
      __l += 4 + val.length();
    }
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.put((byte)(success ? 1 : 0));
    bb.putInt(uids.size());
    for(java.lang.String val : uids) {
      Serialization.writeString(bb, val);
    }
    bb.putInt(recipes.size());
    for(java.lang.String val : recipes) {
      Serialization.writeString(bb, val);
    }
  }

  public void deserialize(ByteBuffer bb) {
    success = bb.get() != 0 ? true : false;

    int __uids_len = bb.getInt();
    uids = new java.util.ArrayList<java.lang.String>(__uids_len);
    for(int __i=0; __i<__uids_len; __i++) {
      uids.add(Serialization.readString(bb));
    }

    int __recipes_len = bb.getInt();
    recipes = new java.util.ArrayList<java.lang.String>(__recipes_len);
    for(int __i=0; __i<__recipes_len; __i++) {
      recipes.add(Serialization.readString(bb));
    }
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Response))
      return false;
    Response other = (Response) o;
    return
      success == other.success &&
      uids.equals(other.uids) &&
      recipes.equals(other.recipes) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.success ? 1231 : 1237);
    result = prime * result + (this.uids == null ? 0 : this.uids.hashCode());
    result = prime * result + (this.recipes == null ? 0 : this.recipes.hashCode());
    return result;
  }
} // class Response

} //class
