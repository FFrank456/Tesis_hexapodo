import processing.core.PApplet;
class TestStepExperiment {
  static void check(boolean value, String reason) { if (!value) throw new AssertionError(reason); }
  static String data(long time, long count, long id, int stage, long start) {
    return "DATA,1,0,0.142,0.000,0.000,1,CW,20.000,0.000,1000,10000,1000,2048,0.100,0,OPEN,0,0,0,20,0,8320,0,0,1,"+
      time+","+count+","+id+","+stage+","+start+","+start;
  }
  public static void main(String[] args) {
    var app=new ControlCurrentN20_StepExperiment();
    check(app.validatedProgram("0:1000; -25.5:2000;100:100;0:100").equals("0.0:1000;-25.5:2000;100.0:100;0.0:100"),"valid signed levels");
    for (String bad:new String[]{"", "20:0", "101:1000", "NaN:1000", "20:1000;", "20:1000;;0:100", "20:1.5", "0:30001", "0:30000;0:30000;0:30000;0:30000;0:100"}) {
      boolean rejected=false;
      try { app.validatedProgram(bad); } catch (RuntimeException e) { rejected=true; }
      check(rejected,"reject "+bad);
    }
    app.program=app.new InputField(0,0,100,100,"program","",480);
    app.activeInput=app.program;
    for (char c:"-20:1000;0:1000".toCharArray()) { app.key=c; app.keyPressed(); }
    app.key=PApplet.ENTER; app.keyPressed();
    check(app.program.value.equals("-20:1000;0:1000") && app.activeInput==null,"keyboard dispatch");
    app.awaitingAck=true; app.captureProgram="0:1000;20:1000"; app.captureCpr=1000;
    app.processLine("ACK,SEQ,7,2");
    check(app.running && !app.awaitingAck,"ACK starts capture");
    // Count deliberately beyond float's exact integer range.
    app.processLine(data(1000000,20000001,7,0,1000000));
    app.processLine(data(1050000,20000051,7,0,1000000));
    check(app.samples.size()==2 && app.samples.get(0).encoderCount==20000001L,"exact count preserved");
    check(Math.abs(app.samples.get(1).rpm-60)<0.001,"RPM from ESP32 time and output CPR");
    app.processLine(data(1050000,20000051,7,0,1000000));
    app.processLine(data(1100000,0,8,0,1000000));
    check(app.samples.size()==2,"ignore duplicate and foreign sequence");
    app.processLine(data(3000000,20000100,7,-1,1000000));
    check(!app.running && app.outcome.equals("terminado"),"firmware end stops capture");
    var table=app.experimentTable();
    check(table.getRowCount()==3 && table.getString(0,"encoder_count_x4").equals("20000001"),"CSV exact integers");
    app.captureCpr=0;
    check(app.experimentTable().getString(0,"output_position_rev").isEmpty(),"unknown CPR leaves position blank");
    check(table.getString(0,"current_valid").isEmpty() && table.getString(0,"current_status").equals("-1"),"legacy current is unverified");
    app.samples.clear(); app.running=true; app.lastSampleUs=-1;
    String valid=data(4000000,20000200,7,1,1000000).replace("0.000,0.000,1,CW","0.123,0.120,1,CW")+
      ",1,1,6,12,70,100,3,10000,1000,850";
    String invalid=data(4010000,20000250,7,1,1000000).replace("0.000,0.000,1,CW","nan,nan,1,CW")+
      ",0,4,6,12,70,110,3,10000,1000,850";
    app.processLine(valid); app.processLine(invalid);
    check(app.samples.size()==2,"invalid current does not discard encoder/PWM samples");
    check(app.samples.get(1).encoderCount==20000250L && app.samples.get(1).pwm==20,"invalid current preserves other signals");
    check(Float.isNaN(app.samples.get(1).current) && app.samples.get(0).sensing.valid,"new validity applied");
    table=app.experimentTable();
    check(table.getString(0,"current_valid").equals("1") && table.getString(1,"current_valid").equals("0"),"CSV current flags");
    check(table.getString(1,"current_raw_a").isEmpty() && table.getString(1,"current_ema_a").isEmpty(),"CSV invalid current blank");
    check(table.getString(1,"sample_missed_count").equals("3") && table.getString(1,"current_status").equals("4"),"CSV diagnostics preserved");
    check(table.getString(0,"pwm_frequency_observed_hz").equals("10000.0"),"observed rate exported separately");
    for (int code=0;code<=8;code++) {
      if (code==1) continue;
      String frame=data(4020000+code*10000,20000300+code,7,1,1000000)+",0,"+code+",0,-1,70,120,3,10000,1000,0";
      app.processLine(frame);
      check(Float.isNaN(app.latest.current) && !app.latest.sensing.reason().isEmpty(),"invalid reason "+code+" overrides finite stale current");
    }
    int before=app.samples.size();
    app.processLine(valid.substring(0,valid.lastIndexOf(',')));
    check(app.samples.size()==before,"partial diagnostics discarded");
    app.running=true; app.processLine("HEADER,time_ms");
    check(!app.running && !app.firmwareReady,"reset aborts capture");
    System.out.println("PASS: multilevel, keyboard, ACK, timestamps, encoder, RPM, legacy/new CSV, invalid current, diagnostics, reset/finish");
  }
}
