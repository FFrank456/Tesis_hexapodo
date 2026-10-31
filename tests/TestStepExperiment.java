import processing.core.PApplet;
class TestStepExperiment {
  static void check(boolean value,String reason) { if (!value) throw new AssertionError(reason); }
  static String data(long time,long count,long id,int stage,long start) {
    String[] f=TelemetryFixture.fields();
    f[24]=Long.toString(time); f[25]=Long.toString(count); f[26]=Long.toString(id);
    f[27]=Integer.toString(stage); f[28]=f[29]=Long.toString(start); f[50]=Long.toString(time+123);
    return String.join(",",f);
  }
  // Deliberately independent encoder clock/angle: the UI must not reconstruct them.
  static String encoderFrame(long time,long count,int stage,String speed,int valid,long ref) {
    String[] f=PApplet.split(data(time,count,7,stage,1000000),',');
    f[53]=speed; f[54]=Integer.toString(valid); f[55]=valid==1 ? "50017" : "0"; f[58]=Long.toString(ref);
    return String.join(",",f);
  }
  static class Commands extends ControlCurrentN20_StepExperiment {
    String sent="";
    @Override public void sendCommand(String command) { sent=command; }
  }
  public static void main(String[] args) {
    check(TelemetryFixture.fields().length==59,"fixture v6 has 59 fields");
    var app=new ControlCurrentN20_StepExperiment(); app.initControls();
    check(app.programDurationUs(app.validatedProgram(app.rampProgram()))==41000000,"41-stage ramp");
    check(app.validatedProgram("0:1000; -25.5:2000;100:100;0:100").equals("0.0:1000;-25.5:2000;100.0:100;0.0:100"),"signed levels");
    for (String bad:new String[]{"","20:0","101:1000","NaN:1000","20:1000;","20:1000;;0:100","20:1.5","0:30001","0:30000;0:30000;0:30000;0:30000;0:100"}) {
      boolean rejected=false;
      try { app.validatedProgram(bad); } catch (RuntimeException e) { rejected=true; }
      check(rejected,"reject "+bad);
    }
    app.program=app.new InputField(0,0,100,100,"program","",480); app.activeInput=app.program;
    for (char c:"-20:1000;0:1000".toCharArray()) { app.key=c; app.keyPressed(); }
    app.key=PApplet.ENTER; app.keyPressed();
    check(app.program.value.equals("-20:1000;0:1000") && app.activeInput==null,"keyboard dispatch");
    app.awaitingAck=true; app.captureProgram="0:1000;20:1000";
    app.processLine("ACK,SEQ,7,2");
    check(app.running && !app.awaitingAck,"ACK starts capture");
    long count=9007199254740993L;
    app.processLine(encoderFrame(1000000,count,0,"nan",0,3));
    app.processLine(encoderFrame(1050000,count+50,0,"-7.123456789",1,3));
    check(app.samples.size()==2 && app.samples.get(0).encoderCount==count,"exact count preserved");
    check(Double.isNaN(app.samples.get(0).encoder.speedRadS),"invalid speed stays unavailable");
    check(app.samples.get(1).encoder.speedRadS==-7.123456789,"speed is received, not differentiated");
    check(app.samples.get(1).encoder.positionRad==-1.234567891,"position is received, not converted");
    check(app.samples.get(1).encoder.timeUs==1050123 && app.samples.get(1).encoder.speedWindowUs==50017,"independent encoder clock/window");
    app.processLine(encoderFrame(1050000,count+50,0,"-7.123456789",1,3));
    app.processLine(data(1100000,0,8,0,1000000));
    check(app.samples.size()==2,"ignore duplicate and foreign sequence");
    app.processLine(encoderFrame(3000000,count+100,-1,"0.0",1,3));
    check(!app.running && app.outcome.equals("terminado"),"firmware end stops capture");
    var table=app.experimentTable();
    check(table.getRowCount()==3 && table.getString(0,"encoder_count_x4").equals(Long.toString(count)),"CSV exact raw integer");
    check(table.getString(0,"output_speed_rad_s").isEmpty(),"CSV unavailable speed blank");
    check(table.getString(1,"output_position_rad").equals("-1.234567891") && table.getString(1,"output_speed_rad_s").equals("-7.123456789"),"CSV SI pass-through");
    check(table.getString(1,"encoder_reference_count_x4").equals(Long.toString(count)) && table.getString(1,"encoder_relative_count_x4").equals("-140"),"CSV reference/raw received");
    check(table.getString(1,"encoder_speed_window_us").equals("50017") && table.getString(1,"counts_x4_per_output_rev").equals("2800"),"CSV firmware calibration/time");
    check(table.getString(1,"current_mean_a").equals("0.1") && table.getString(1,"current_raw_a").equals("0.0"),"CSV distinguishes block mean/raw diagnostic");
    check(table.getColumnCount()==new java.util.HashSet<String>(java.util.Arrays.asList(table.getColumnTitles())).size(),"CSV columns unique");
    var incompatible=new ControlCurrentN20_StepExperiment();
    incompatible.processLine(TelemetryFixture.with(41,"5"));
    check(incompatible.latest==null && !incompatible.firmwareReady && incompatible.status.contains("Actualiza firmware"),"reject former protocol with update notice");
    incompatible.processLine("DATA,0,0");
    check(!incompatible.firmwareReady && incompatible.latest==null,"short frame rejected safely");
    var invalid=new ControlCurrentN20_StepExperiment();
    invalid.running=true; invalid.activeSequenceId=7;
    String[] f=PApplet.split(encoderFrame(1000000,count,0,"0",1,3),',');
    f[30]="0"; f[31]="10"; f[47]="nan";
    invalid.processLine(String.join(",",f));
    check(invalid.samples.size()==1 && Float.isNaN(invalid.plotValue(invalid.latest,0)),"invalid current preserves time/encoder/PWM as gap");
    check(invalid.experimentTable().getString(0,"current_mean_a").isEmpty(),"invalid mean exports blank");
    String frame=encoderFrame(3100000,0,0,"1.0",1,3);
    app.processLine(frame.substring(0,frame.lastIndexOf(',')));
    check(!app.firmwareReady && app.latest==null,"incomplete frame rejected");
    String[] fields=PApplet.split(frame,','); fields[49]="2";
    app.processLine(String.join(",",fields));
    check(!app.latest.encoder.supported && Double.isNaN(app.latest.encoder.speedRadS),"unknown encoder version unavailable");
    app.running=true; app.processLine("HEADER,time_ms");
    check(!app.running && !app.firmwareReady,"reset aborts capture");
    var changing=new ControlCurrentN20_StepExperiment();
    changing.running=true; changing.activeSequenceId=7;
    changing.processLine(encoderFrame(1000000,count,0,"0",1,3));
    changing.processLine(encoderFrame(1050000,count,0,"0",1,4));
    check(!changing.running && changing.samples.size()==1,"reference change aborts without mixed CSV origins");
    var zero=new Commands();
    zero.processLine(encoderFrame(1000000,count,0,"2.0",1,3));
    zero.zeroEncoder();
    check(zero.sent.equals("ENCODER,ZERO") && zero.latest.encoder.positionRad==-1.234567891,"zero requests firmware; no local offset");
    zero.processLine("ACK,ENCODER,ZERO");
    check(zero.status.contains("confirmado"),"zero ACK exact protocol");
    zero.sent=""; zero.running=true; zero.zeroEncoder();
    check(zero.sent.isEmpty(),"zero blocked during sequence");
    System.out.println("PASS: v6 steps, mean/CSV, encoder SI, invalid frames, reference changes and zero command");
  }
}
