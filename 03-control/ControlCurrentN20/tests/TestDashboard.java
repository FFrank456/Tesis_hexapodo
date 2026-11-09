import java.util.*;
import java.nio.file.*;
class TestDashboard extends TestCurrentPI {
  void frame(long us,String mode,float measurement) {
    String[] f=TelemetryFixture.fields();
    f[24]=Long.toString(us);f[50]=f[24];f[14]=mode;
    f[15]="0.1";f[47]=Float.toString(measurement);f[18]="40";
    processLine(String.join(",",f));
  }
  public static void main(String[] args) {
    try { run(); } catch(Exception e) { throw new RuntimeException(e); }
  }
  static void run() throws Exception {
    Locale.setDefault(Locale.forLanguageTag("es-PE"));
    var a=new TestDashboard(); a.initialize();
    a.processLine("ACK,LIMIT,0.000,40.000");
    for(int n=0;n<=500;n++)a.frame(1000000+n*10000,"PID",.08f);
    var m=a.metrics(a.samples,500,false);
    check(Math.abs(m.seconds-5)<1e-6,"5 seconds of device time");
    check(Math.abs(m.rmse()-.02)<1e-6 && Math.abs(m.mae()-.02)<1e-6,"known current error metrics");
    check(Math.abs(m.peak()-.02)<1e-6 && m.saturation()==100,"peak and requested ceiling");
    a.frame(6020000,"PID",Float.NaN);
    check(a.metrics(a.samples,501,false).seconds<5,"invalid current excludes intervals");
    a.frame(6040000,"OPEN",.08f);
    check(Float.isNaN(a.metrics(a.samples,502,false).rmse()),"OPEN has no controller metrics");
    a.processLine("SPEED,1,6050000,-1,10,2,0.04,0.0284,0.01392,0.001,0.1,0,6050000");
    a.frame(6050000,"SPEED",.04f);
    var last=a.samples.get(503); last.encoder.speedRadS=-8;
    check(a.loopReference(last,true)==-10 && a.loopError(last,true)==-2,"signed speed units");
    a.frame(6060000,"SPEED",.04f);a.samples.get(504).encoder.speedRadS=-8;
    check(Math.abs(a.metrics(a.samples,504,true).rmse()-2)<1e-6,"speed metrics not blended with current");
    a.frame(1000000,"PID",.08f);
    check(a.samples.get(505).segment!=a.samples.get(504).segment,"clock restart splits segments");
    check(Float.isNaN(a.samples.get(505).pwmLimit),"clock reset invalidates confirmed limit");
    check(a.metrics(a.samples,505,false).seconds==0,"clock restart cannot bridge metrics");
    var table=a.sampleTable(a.samples);
    check(table.getString(0,"loop_unit").equals("A_eq_CS") && table.getString(504,"loop_unit").equals("rad/s"),"one schema both modes");
    check(table.getInt(502,"loop_valid")==0 && table.getString(502,"loop_error").isEmpty(),"OPEN empty error");
    check(table.getInt(505,"csv_schema_version")==3 && table.getString(505,"adc_time_us").equals("1000000"),"schema and exact source clock");
    Path path=Path.of("build-codex/dashboard-export-test.csv");
    a.writeCsvSnapshot(a.samples,path.toFile());
    var restored=new processing.data.Table(path.toFile(),"header,csv");
    check(restored.getRowCount()==506 && Math.abs(restored.getFloat(500,"rmse_5s")-.02)<1e-6,"CSV round trip with Spanish locale");
    a.clearSamples();for(int n=0;n<60002;n++)a.frame(1000000+n*10000L,"PID",.08f);
    check(a.samples.size()==60000 && a.recordFull && a.graphSamples.size()==2200,"bounded full recording and independent live view");
    check(a.samples.get(0).adcTimeUs==1000000,"old recorded rows not silently discarded");
    a.clearSamples();check(a.samples.isEmpty() && a.graphSamples.isEmpty() && !a.recordFull,"new recording reset");
    // Tabs are views, not motor commands.
    a.width=1366;a.height=768;a.initControls();a.commands.clear();
    a.mouseX=200;a.mouseY=100;a.mousePressed();
    check(a.speedPage && a.commands.isEmpty(),"speed tab does not actuate");
    System.out.println("PASS: dashboard metrics, clock gaps, modes, CSV round trip, recording cap and view-only tabs");
  }
}
