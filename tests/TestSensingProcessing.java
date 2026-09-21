import processing.awt.PGraphicsJava2D;
import processing.core.PApplet;

class TestSensingProcessing {
  static void check(boolean condition,String why) { if (!condition) throw new AssertionError(why); }
  static String data(int valid,int status,String current) {
    return "DATA,1234,0,nan,"+current+","+current+",1,CW,40,0,100,10000,1000,20,0.1,0,OPEN,0.1,nan,0,40,0,8320,0,0,1,1234000,55,1,1,1000000,1200000,"+
      valid+","+status+",5,10,20,100,2,10000,1000,900";
  }
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
    manual.processLine(data(1,1,"0.123"));
    check(manual.sensing.valid && Math.abs(manual.currentRawA-.123)<.001,"manual valid current");
    manual.processLine(data(0,3,"nan"));
    check(Float.isNaN(manual.currentRawA) && Float.isFinite(manual.currentAxisMaxA),"manual invalid current leaves scale finite");
    manual.processLine(data(1,1,"0.120"));
    manual.drawHistoryLine(manual.currentRawHistory,0,1,0,0,200,100,0);
    check(manual.segments==2 && manual.points==2,"manual breaks curve at invalid point");
    String legacy=data(1,1,"0.120"); legacy=legacy.substring(0,legacy.indexOf(",1,1,5,10"));
    manual.processLine(legacy);
    check(!manual.sensing.present && !manual.sensing.valid && Float.isFinite(manual.currentRawA),"manual legacy unverified");

    var pid=new PIDPlot(); graphics(pid);
    pid.processLine(data(1,1,"0.123")); pid.processLine(data(0,4,"nan")); pid.processLine(data(1,1,"0.120"));
    check(pid.samples.size()==3 && Float.isNaN(pid.samples.get(1).currentEmaA),"PID keeps invalid sample as gap");
    check(Float.isFinite(pid.currentAxisMax(pid.samples)),"PID finite current axis");
    pid.drawSamplesLine(pid.samples,0,0,200,100,1,0,0);
    check(pid.segments==2 && pid.points==2,"PID breaks curve at invalid point");
    var table=pid.sampleTable(pid.samples);
    check(table.getString(1,"current_raw_a").isEmpty() && table.getString(1,"current_valid").equals("0"),"PID CSV invalid reading blank and flagged");
    pid.processLine("ERR,PID,CURRENT_INVALID");
    check(pid.status.equals("ERR,PID,CURRENT_INVALID"),"PID rejection visible");

    var step=new StepPlot(); graphics(step);
    for (int i=0;i<3;i++) {
      var sample=step.new Sample(PApplet.split(data(i==1 ? 0 : 1,i==1 ? 4 : 1,i==1 ? "nan" : "0.123"),','),0);
      sample.elapsedUs=i*10000; step.samples.add(sample);
    }
    step.drawPlot(0,0,640,130,0,"Corriente");
    check(step.segments==4 && step.points==4,"step raw and EMA each break at invalid point");
    String continuous=data(1,9,"0.057")+",3,0.142";
    manual.processLine(continuous);
    check(manual.sensing.valid && manual.sensing.dmaOverflows==3,"continuous manual diagnostics");
    check(Math.abs(manual.sensing.voltageEma-.142)<.00001,"EMA voltage preserved");
    pid.processLine(continuous);
    check(pid.sensing.valid,"PID sketch receives continuous diagnostics without enabling firmware PID");
    var cs=step.new Sample(PApplet.split(continuous,','),0);
    check(cs.sensing.valid && Float.isFinite(cs.ema),"continuous zero-floor values are retained");
    step.samples.clear(); step.samples.add(cs);
    var continuousTable=step.experimentTable();
    check(continuousTable.getString(0,"measurement_basis").equals("continuous_CS_all_phases"),"CSV identifies whole-CS estimate");
    check(continuousTable.getString(0,"dma_overflows").equals("3"),"CSV overflow counter");
    manual.processLine(data(0,10,"nan")+",4,nan");
    check(!manual.sensing.valid && Float.isNaN(manual.currentRawA),"overflow leaves gap");
    String sync=data(1,1,"0.100")+",0,0.249";
    manual.processLine(sync); pid.processLine(sync);
    check(manual.sensing.valid && manual.sensing.readUs==5 && manual.sensing.phaseUs==10,"ON timing decoded");
    check(Math.abs(manual.sensing.voltageEma-.249)<.00001,"ON EMA voltage decoded");
    var on=step.new Sample(PApplet.split(sync,','),0);
    step.samples.clear(); step.samples.add(on);
    check(step.experimentTable().getString(0,"measurement_basis").equals("validated_ON_current"),"ON CSV basis");
    var pidOn=pid.sampleTable(pid.samples);
    check(pidOn.getString(pidOn.getRowCount()-1,"measurement_basis").equals("validated_ON_current"),"PID ON CSV basis");
    manual.processLine(data(0,3,"nan")+",0,nan");
    check(!manual.sensing.valid && Float.isNaN(manual.currentEmaA),"short ON leaves EMA gap");
    String[] lowFields=PApplet.split(data(1,11,"0.0")+",0,0.249",',');
    lowFields[3]="0.124"; lowFields[5]="0.100";
    String low=String.join(",",lowFields);
    manual.processLine(low); pid.processLine(low);
    check(manual.sensing.valid && manual.currentRawA==0 && manual.currentEmaA>0,"low ADC zero retains EMA");
    check(pid.sensing.valid && pid.currentRawA==0,"PID displays below-range zero");
    var lowSample=step.new Sample(PApplet.split(low,','),0);
    step.samples.clear(); step.samples.add(lowSample);
    check(lowSample.sensing.valid && lowSample.current==0 && lowSample.ema>0,"step below-range zero");
    check(step.experimentTable().getString(0,"measurement_basis").equals("ON_below_range_zero"),"CSV labels censored current");
    manual.processLine(data(0,3,"nan")+",0,nan");
    check(Float.isNaN(manual.currentRawA),"timing failure is not a zero");
    String dma=data(1,13,"0.100")+",0,0.249,2,1200,800,0";
    manual.processLine(dma); pid.processLine(dma);
    check(manual.sensing.valid && manual.sensing.version==2 && manual.sensing.rejectedWindow==1200,"DMA version and counters");
    check(pid.sensing.valid && pid.sensing.reason().contains("100%"),"DMA DC label");
    var ds=step.new Sample(PApplet.split(dma,','),0);
    step.samples.clear(); step.samples.add(ds);
    check(ds.sensing.valid,"DMA step valid");
    check(step.experimentTable().getString(0,"measurement_basis").equals("DMA_DC_100_percent"),"DMA CSV DC basis");
    check(step.experimentTable().getString(0,"dma_rejected_quality").equals("800"),"DMA CSV rejects");
    String phaseMissing=data(0,12,"nan")+",0,nan,2,0,200000,0";
    manual.processLine(phaseMissing); pid.processLine(phaseMissing);
    check(!manual.sensing.valid && Float.isNaN(manual.currentRawA),"unverified phase never zero current");
    check(!pid.sensing.valid && Float.isNaN(pid.currentEmaA),"unverified phase no stale EMA");
    var noPhase=step.new Sample(PApplet.split(phaseMissing,','),0);
    check(!noPhase.sensing.valid && Float.isNaN(noPhase.current),"step unverified phase gap");
    manual.processLine(data(1,13,"0.100")+",0,0.249");
    check(!manual.sensing.valid,"new status requires new protocol version");
    String freeDma=data(1,9,"0.057")+",0,0.142,3,0,0,0";
    manual.processLine(freeDma); pid.processLine(freeDma);
    check(manual.sensing.valid && manual.sensing.version==3 && !manual.sensing.phaseValidated,"continuous DMA needs no phase");
    check(pid.sensing.valid,"PID interface receives CS without enabling control");
    var freeSample=step.new Sample(PApplet.split(freeDma,','),0);
    step.samples.clear(); step.samples.add(freeSample);
    check(step.experimentTable().getString(0,"measurement_basis").equals("continuous_CS_all_phases"),"version 3 CSV basis");
    String[] zeroFields=PApplet.split(data(1,15,"0.0")+",0,0.0249,4,0,0,0",',');
    zeroFields[3]="0.142"; zeroFields[5]="0.010";
    String zeroAssigned=String.join(",",zeroFields);
    manual.processLine(zeroAssigned); pid.processLine(zeroAssigned);
    check(manual.sensing.valid && manual.currentRawA==0 && manual.currentEmaA>0,"zero assignment retains EMA");
    check(Math.abs(manual.adcVoltage-.142)<.00001 && pid.sensing.valid,"SDK voltage retained");
    var zs=step.new Sample(zeroFields,0); step.samples.clear(); step.samples.add(zs);
    check(zs.sensing.valid && step.experimentTable().getString(0,"raw_zero_assigned").equals("1"),"assigned zero exported");
    // Last raw sample is zero, but the block mean includes positive samples before it.
    String blockMean=zeroAssigned.replace(",4,0,0,0",",5,0,0,0")+",250.5,0.249,0.100,200";
    manual.processLine(blockMean); pid.processLine(blockMean);
    var meanSample=step.new Sample(PApplet.split(blockMean,','),0);
    check(manual.currentRawA==0 && Math.abs(manual.currentDisplayA-.100)<.00001,"manual plots mean despite last raw zero");
    check(manual.sensing.valid && manual.sensing.dmaBlockSamples==200,"complete mean block valid");
    check(Math.abs(manual.currentRawHistory[manual.historyIndex-1]-.100)<.00001,"manual history uses block mean");
    check(Math.abs(pid.sensing.primaryCurrent(pid.currentRawA)-.100)<.00001,"PID display uses block mean");
    check(Math.abs(step.plotValue(meanSample,0)-.100)<.00001 && meanSample.current==0,"step plots mean preserves legacy raw");
    step.samples.clear(); step.samples.add(meanSample);
    var meansTable=step.experimentTable();
    check(meansTable.getString(0,"measurement_basis").equals("dma_block_mean_CS_raw_zero_assigned"),"v5 mean basis");
    check(meansTable.getString(0,"raw_zero_assigned").equals("1") && meansTable.getString(0,"current_mean_a").equals("0.1"),"zero flag belongs only to last sample");
    check(meansTable.getString(0,"adc_raw_mean").equals("250.5") && meansTable.getString(0,"adc_voltage_mean").equals("0.249") && meansTable.getString(0,"dma_block_samples").equals("200"),"mean CSV columns");
    var pidMeans=pid.sampleTable(pid.samples);
    check(pidMeans.getString(pidMeans.getRowCount()-1,"current_mean_a").equals("0.1"),"PID CSV mean");
    for (String badMean:new String[]{blockMean.replace("0.249,0.100,200","0.249,0.100,199"),
        blockMean.replace("0.249,0.100,200","0.249,nan,200"),
        blockMean.substring(0,blockMean.lastIndexOf(',')),
        blockMean.replace(",1,15,5,10",",0,15,5,10")}) {
      manual.processLine(badMean); pid.processLine(badMean);
      var badSample=step.new Sample(PApplet.split(badMean,','),0);
      check(!manual.sensing.valid && Float.isNaN(manual.currentDisplayA),"manual mean invalid never falls back");
      check(!pid.sensing.valid && Float.isNaN(pid.sensing.primaryCurrent(pid.currentRawA)),"PID invalid mean gap");
      check(!badSample.sensing.valid && Float.isNaN(step.plotValue(badSample,0)),"step invalid mean gap");
      step.samples.clear(); step.samples.add(badSample);
      check(step.experimentTable().getString(0,"current_mean_a").isEmpty(),"invalid mean blank in CSV");
    }
    manual.processLine(zeroAssigned); pid.processLine(zeroAssigned);
    check(manual.currentDisplayA==0 && manual.sensing.version==4,"v5 to v4 clears mean selection");
    System.out.println("PASS: legacy/ON/DMA/free-running/assigned-zero/block-mean parsing, selection, gaps and CSV");
  }
}
