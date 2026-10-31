import java.util.Arrays;
import processing.awt.PGraphicsJava2D;
import processing.core.PApplet;

/** Exact v6 block-mean protocol and display-only encoder regression coverage. */
class TestSensingProcessing {
  static void check(boolean condition,String why) { if (!condition) throw new AssertionError(why); }
  static String data(int valid,int status,String current) { return TelemetryFixture.reading(valid,status,current); }
  static void graphics(PApplet app) {
    var g=new PGraphicsJava2D(); g.setParent(app); g.setSize(1180,760); app.width=1180; app.height=760; app.g=g;
    g.beginDraw(); app.textFont(app.createFont("Arial",14));
  }
  static class ManualPlot extends ControlCurrentN20_Processing4 {
    int segments,points;
    @Override public void beginShape() { segments++; }
    @Override public void endShape() { }
    @Override public void vertex(float x,float y) { check(Float.isFinite(x)&&Float.isFinite(y),"manual finite vertex"); points++; }
  }
  static class PIDPlot extends ControlCurrentN20_CurrentPID {
    int segments,points;
    @Override public void beginShape() { segments++; }
    @Override public void endShape() { }
    @Override public void vertex(float x,float y) { check(Float.isFinite(x)&&Float.isFinite(y),"PID finite vertex"); points++; }
  }
  static class StepPlot extends ControlCurrentN20_StepExperiment {
    int segments,points;
    @Override public void beginShape() { segments++; }
    @Override public void endShape() { }
    @Override public void vertex(float x,float y) { check(Float.isFinite(x)&&Float.isFinite(y),"step finite vertex"); points++; }
  }
  public static void main(String[] args) {
    var manual=new ManualPlot(); graphics(manual);
    manual.processLine(data(1,15,"0.123"));
    check(manual.sensing.valid && manual.currentRawA==0 && Math.abs(manual.currentDisplayA-.123)<.00001,"manual uses mean despite last raw zero");
    check(Math.abs(manual.adcVoltage-.142)<.00001 && Math.abs(manual.sensing.adcVoltageMean-.249)<.00001,"SDK diagnostic differs from mean voltage");
    check(manual.sensing.version==6 && manual.sensing.dmaBlockSamples==200,"complete v6 block received");
    manual.processLine(data(0,10,"nan"));
    check(Float.isNaN(manual.currentDisplayA) && Float.isFinite(manual.currentAxisMaxA),"manual invalid current leaves scale finite");
    check(manual.encoder.supported && manual.dutyCw==40,"measurement gap retains encoder/PWM");
    manual.processLine(data(1,9,"0.120"));
    manual.drawHistoryLine(manual.currentMeanHistory,0,1,0,0,200,100,0);
    check(manual.segments==2 && manual.points==2,"manual one mean curve breaks at invalid point");
    check(Math.abs(manual.currentMeanHistory[manual.historyIndex-1]-.120)<.00001,"manual history receives mean directly");

    var pid=new PIDPlot(); graphics(pid);
    pid.processLine(data(1,15,"0.123")); pid.processLine(data(0,10,"nan")); pid.processLine(data(1,9,"0.120"));
    check(pid.samples.size()==3 && Float.isNaN(pid.samples.get(1).currentMeanA),"PID keeps unavailable measurement as gap");
    check(pid.samples.get(1).encoder.supported && pid.samples.get(1).dutyPercent==40,"PID gap preserves encoder/PWM");
    check(Float.isFinite(pid.currentAxisMax(pid.samples)),"PID finite current axis");
    pid.drawSamplesLine(pid.samples,0,0,200,100,1,0,0);
    check(pid.segments==2 && pid.points==2,"PID one mean curve breaks at invalid point");
    var table=pid.sampleTable(pid.samples);
    check(table.getString(1,"current_mean_a").isEmpty() && table.getString(1,"current_valid").equals("0"),"PID CSV invalid mean blank and flagged");
    check(table.getString(0,"current_raw_a").equals("0.0") && table.getString(0,"current_mean_a").equals("0.123"),"PID CSV separates diagnostic and mean");
    check(table.getColumnCount()==new java.util.HashSet<String>(Arrays.asList(table.getColumnTitles())).size(),"PID CSV columns unique");
    pid.processLine("ERR,PID,CURRENT_INVALID");
    check(pid.status.equals("ERR,PID,CURRENT_INVALID"),"PID rejection visible");

    var step=new StepPlot(); graphics(step);
    for (int i=0;i<3;i++) {
      var sample=step.new Sample(PApplet.split(data(i==1 ? 0 : 1,i==1 ? 10 : 9,i==1 ? "nan" : "0.123"),','),0);
      sample.elapsedUs=i*10000; step.samples.add(sample);
    }
    step.drawPlot(0,0,640,130,0,"Corriente");
    check(step.segments==2 && step.points==2,"step draws one current curve with invalid gap");
    var meanSample=step.new Sample(TelemetryFixture.fields(),0);
    check(Math.abs(step.plotValue(meanSample,0)-.100)<.00001 && meanSample.current==0,"step uses block mean, preserving diagnostic zero");
    step.samples.clear(); step.samples.add(meanSample);
    var meansTable=step.experimentTable();
    check(meansTable.getString(0,"measurement_basis").equals("dma_block_mean_CS_raw_zero_assigned"),"CSV identifies measurement basis");
    check(meansTable.getString(0,"raw_zero_assigned").equals("1") && meansTable.getString(0,"current_mean_a").equals("0.1"),"zero flag applies to diagnostic, not whole mean");
    check(meansTable.getString(0,"adc_raw_mean").equals("250.5") && meansTable.getString(0,"adc_voltage_mean").equals("0.249") && meansTable.getString(0,"dma_block_samples").equals("200"),"mean CSV values pass through");
    for (String badMean:new String[]{TelemetryFixture.with(48,"199"),TelemetryFixture.with(47,"nan"),
        TelemetryFixture.with(46,"nan"),TelemetryFixture.with(45,"nan"),TelemetryFixture.with(30,"0"),TelemetryFixture.with(31,"10")}) {
      manual.processLine(badMean); pid.processLine(badMean);
      var badSample=step.new Sample(PApplet.split(badMean,','),0);
      check(!manual.sensing.valid && Float.isNaN(manual.currentDisplayA),"manual invalid mean never falls back to raw");
      check(!pid.sensing.valid && Float.isNaN(pid.currentMeanA),"PID invalid mean never falls back");
      check(!badSample.sensing.valid && Float.isNaN(step.plotValue(badSample,0)),"step invalid mean gap");
      check(manual.encoder.supported && pid.encoder.supported && badSample.encoder.supported,"invalid mean leaves encoder available");
      step.samples.clear(); step.samples.add(badSample);
      check(step.experimentTable().getString(0,"current_mean_a").isEmpty(),"invalid mean blank in CSV");
    }
    String[] zero=TelemetryFixture.fields(); zero[45]="0"; zero[46]="0"; zero[47]="0";
    manual.processLine(String.join(",",zero)); pid.processLine(String.join(",",zero));
    check(manual.sensing.valid && manual.currentDisplayA==0 && pid.currentMeanA==0,"valid zero mean is distinct from missing measurement");

    String encoderFrame=TestStepExperiment.encoderFrame(1000000,9007199254740993L,0,"-7.123456789",1,3);
    manual.processLine(encoderFrame); pid.processLine(encoderFrame);
    check(manual.encoder.positionRad==-1.234567891 && manual.encoder.speedRadS==-7.123456789,"manual SI pass-through");
    check(pid.encoder.positionRad==-1.234567891 && pid.encoder.speedRadS==-7.123456789,"PID SI pass-through");
    var encoderTable=pid.sampleTable(pid.samples); int last=encoderTable.getRowCount()-1;
    check(encoderTable.getString(last,"output_speed_rad_s").equals("-7.123456789") &&
      encoderTable.getString(last,"encoder_count_x4").equals("9007199254740993"),"PID CSV preserves encoder precision");
    String noSpeed=TestStepExperiment.encoderFrame(1050000,9007199254741000L,0,"nan",0,3);
    manual.processLine(noSpeed); pid.processLine(noSpeed);
    check(manual.encoder.supported && Double.isNaN(manual.encoder.speedRadS) && Double.isNaN(pid.encoder.speedRadS),"speed unavailable remains unavailable");
    String unknownEncoder=TelemetryFixture.with(49,"2"); manual.processLine(unknownEncoder); pid.processLine(unknownEncoder);
    check(!manual.encoder.supported && !pid.encoder.supported && manual.sensing.valid && pid.sensing.valid,"unknown encoder version cannot invent SI");
    for (String incompatible:new String[]{"DATA,0,0",TelemetryFixture.with(41,"5"),TelemetryFixture.with(41,"7"),
        encoderFrame.substring(0,encoderFrame.lastIndexOf(',')),String.join(",",Arrays.copyOf(TelemetryFixture.fields(),52)),
        TelemetryFixture.data()+",unexpected"}) {
      manual.processLine(TelemetryFixture.data()); pid.processLine(TelemetryFixture.data());
      int rows=pid.samples.size(); manual.processLine(incompatible); pid.processLine(incompatible);
      check(!manual.sensing.valid && Float.isNaN(manual.currentDisplayA) && !manual.encoder.supported,"manual rejects incompatible frame and clears live values");
      check(!pid.sensing.valid && Float.isNaN(pid.currentMeanA) && !pid.encoder.supported,"PID rejects incompatible frame and clears live values");
      check(pid.samples.size()==rows && pid.status.contains("incompatible"),"PID structural rejection not recorded as measurement");
    }
    System.out.println("PASS: v6 mean-only displays, gaps/CSV, protocol rejection and ESP32 encoder SI");
  }
}
