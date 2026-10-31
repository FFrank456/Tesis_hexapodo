import java.util.*;
class TestPositionUI extends TestCurrentPI {
  static final String POS="POSITION,1,2000000,1.57,0.2,0.4,6.166,3192,0,2,5,0.02,0,1,2000000,3";
  void fresh(){
    processLine("SPEED,1,2000000,1,0.4,0.1,0.03,0.0284,0.01392,0.01,0.1,0,2000000");
    processLine(POS);
  }
  public static void main(String[] args) {
    Locale.setDefault(Locale.forLanguageTag("es-PE"));
    var a=new TestPositionUI();a.width=1366;a.height=768;a.initControls();a.positionPage=true;
    a.positionRefSlider.value=1.57f;a.controlModeText="OPEN";
    a.sendPositionReference();check(a.commands.isEmpty(),"old firmware cannot activate position");
    a.fresh();a.sendPositionReference();
    check(a.commands.equals(List.of("PLIMIT,2.0000,5.0000,0.0200","LIMIT,0,40.000","ILIMIT,0.1000","PREF,1.57000","MODE,POSITION")),"position activation limits and target precede MODE");
    a.commands.clear();a.controlModeText="POSITION";a.positionRefSlider.value=-1;
    a.sendPositionReference();check(a.commands.contains("PREF,-1.00000") && a.commands.stream().noneMatch(x->x.startsWith("MODE,") || x.startsWith("PLIMIT,")),"live target no mode restart or limit mutation");
    a.commands.clear();a.positionMaxSlider.value=3;a.sendPositionReference();check(a.commands.isEmpty(),"STOP required to change position limits");
    a.positionMaxSlider.value=2;a.sendPositionParameters();check(a.commands.isEmpty(),"no position gain change while active");
    a.activatePi();a.activateSpeed();check(a.commands.isEmpty(),"position exit requires STOP");
    a.controlModeText="OPEN";a.sendPositionParameters();check(a.commands.get(0).equals("PPI,6.1660,3192.0000"),"correct MATLAB PI, no derivative");
    a.commands.clear();a.controlModeText="PID";a.sendPositionReference();check(a.commands.isEmpty(),"position entry requires STOP");
    String[] f=TelemetryFixture.fields();f[14]="POSITION";f[24]="2000000";f[50]="2000000";f[52]="1.37";
    a.fresh();a.processLine(String.join(",",f));
    f[24]="2020000";f[50]="2020000";a.processLine(String.join(",",f));
    check(Math.abs(a.metrics(a.samples,1,2).rmse()-.2)<1e-6,"position error in rad");
    var table=a.sampleTable(a.samples);
    check(table.getInt(0,"csv_schema_version")==3 && table.getString(0,"loop_unit").equals("rad"),"CSV active loop position");
    check(table.getString(0,"position_ref_rad").equals("1.57") && table.getString(0,"position_reference_id").equals("3"),"position source reference preserved");
    a.samples.get(1).encoder.referenceId=4;
    check(Float.isNaN(a.loopReference(a.samples.get(1),2)),"zero reference mismatch invalidates position loop metrics");
    a.processLine(POS.replace(",1.57,",",nan,"));check(!a.positionFresh(),"invalid POSITION rejected");
    a.fresh();a.lastPositionMillis=a.millis()-501;a.controlModeText="OPEN";a.sendPositionReference();check(a.commands.isEmpty(),"stale position cannot activate");
    a.commands.clear();a.mouseX=245;a.mouseY=100;a.mousePressed();check(a.positionPage && a.commands.isEmpty(),"position tab never actuates");
    System.out.println("PASS: position UI commands, MATLAB PI, mode guards, target changes, timestamps, CSV and metrics");
  }
}
