/** Exact DATA v6 fixture. Deliberate differences distinguish raw diagnostics from block means. */
class TelemetryFixture {
  static String[] fields() {
    return new String[]{"DATA","1234","0","0.142","0.000","1","CW","40","0","100",
      "10000","200000","20","0","OPEN","0.1","nan","0","40","0","8320","0","0","1",
      "1234000","55","1","1","1000000","1200000","1","15","5","-1","200000","1000","2",
      "10000","1000","200000","0","6","0","0","0","250.5","0.249","0.100","200",
      "1","1234123","-140","-1.234567891","-7.123456789","1","50017","2800","9007199254740993","3"};
  }
  static String data() { return String.join(",",fields()); }
  static String with(int index,String value) { String[] f=fields(); f[index]=value; return String.join(",",f); }
  static String reading(int valid,int status,String current) {
    String[] f=fields(); f[30]=Integer.toString(valid); f[31]=Integer.toString(status); f[47]=current;
    return String.join(",",f);
  }
}
