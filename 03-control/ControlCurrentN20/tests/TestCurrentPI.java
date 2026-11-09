import java.util.ArrayList;
import java.util.Locale;

class TestCurrentPI extends ControlCurrentN20_CurrentPID {
  final ArrayList<String> commands = new ArrayList<>();
  @Override public void sendCommand(String command) { commands.add(command); }
  static void check(boolean ok, String why) { if (!ok) throw new AssertionError(why); }
  void initialize() {
    kpSlider = new Slider(0,0,100,0,1000,"Kp"); kpSlider.value=pidKp;
    kiSlider = new Slider(0,0,100,0,150000,"Ki"); kiSlider.value=pidKi;
    kdTimes100Slider = new Slider(0,0,100,0,1000,"Kd");
    outputMaxSlider = new Slider(0,0,100,0,100,"limit"); outputMaxSlider.value=40;
    refMilliAmpSlider = new Slider(0,0,100,0,1000,"ref"); refMilliAmpSlider.value=100;
    direction="CW";
    speedRefSlider=new Slider(0,0,100,0,32,"speed"); speedRefSlider.value=10;
    speedKpSlider=new Slider(0,0,100,0,.2f,"speed Kp"); speedKpSlider.value=.0284f;
    speedKiSlider=new Slider(0,0,100,0,.2f,"speed Ki"); speedKiSlider.value=.01392f;
    speedLimitSlider=new Slider(0,0,100,1,1000,"CS limit"); speedLimitSlider.value=100;
  }
  public static void main(String[] args) {
    Locale.setDefault(Locale.forLanguageTag("es-PE"));
    var app=new TestCurrentPI(); app.initialize();
    app.kdTimes100Slider.value=999;
    app.sendPidParameters();
    check(app.commands.get(0).equals("PID,8.8320,80090.0000,0.0000"),"requested PI gains / D disabled");
    app.commands.clear(); app.activatePi();
    check(app.commands.get(0).equals("LIMIT,0,40.000"),"limit before activation");
    check(app.commands.get(1).equals("MODE,PID,CW"),"PI activation");
    app.commands.clear(); app.controlModeText="PID"; app.sendCurrentReference();
    check(app.commands.contains("IREF,0.1000"),"reference in amperes");
    check(app.commands.stream().noneMatch(x->x.startsWith("MODE,")),"reference change must not reinitialize controller");
    app.commands.clear(); app.activateSpeed(); check(app.commands.isEmpty(),"old firmware cannot activate speed");
    app.processLine("SPEED,1,1000000,1,10,2,0.05,0.0284,0.01392,0.001,0.1,0,990000");
    app.activateSpeed();
    check(app.commands.equals(java.util.List.of("LIMIT,0,40.000","ILIMIT,0.1000","WREF,10.0000","MODE,SPEED,CW")),"speed activation ordered after limits and reference");
    app.commands.clear(); app.controlModeText="SPEED"; app.sendSpeedReference();
    check(app.commands.stream().noneMatch(x->x.startsWith("MODE,")),"speed changes preserve controller state");
    app.commands.clear(); app.selectDirection("CCW"); check(app.commands.isEmpty(),"no reversal while running");
    app.sendSpeedParameters(); check(app.commands.get(0).equals("SPI,0.02840,0.01392"),"MATLAB speed gains");
    app.commands.clear(); app.sendCurrentReference();
    check(app.commands.indexOf("MODE,PID,CW")<app.commands.indexOf("IREF,0.1000"),"leave SPEED before manual IREF");
    app.processLine(TelemetryFixture.with(14,"SPEED"));
    app.pidKp=999; // Export must retain per-sample gains, not the last live settings.
    var table=app.sampleTable(app.samples);
    check(table.getString(0,"speed_telemetry_time_us").equals("1000000"),"speed source timestamp retained");
    check(table.getFloat(0,"kp")==0,"per-sample gain");
    app.processLine("SPEED,1,1000000,1,nan,2,0.05,0.0284,0.01392,0.001,0.1,0,990000");
    check(!app.speedFresh(),"nonfinite telemetry invalidates speed");
    app.commands.clear(); app.activateSpeed(); check(app.commands.isEmpty(),"invalid telemetry blocks activation");
    app.processLine("SPEED,1,1000000,-1,10,2,0.05,0.0284,0.01392,0.001,0.1,1,990000");
    app.lastSpeedMillis=app.millis()-501; app.activateSpeed(); check(app.commands.isEmpty(),"stale telemetry blocks activation");
    System.out.println("PASS: Processing PI gains, zero Kd, limit before activation, reference without mode reset");
  }
}
