import processing.core.PApplet;
class RenderDMA {
  static String frame(int status) {
    boolean valid=status==13;
    String[] f=PApplet.split(TestSensingProcessing.data(valid ? 1 : 0,status,valid ? "0.100" : "nan")+
      ",0,"+(valid ? "0.249" : "nan")+",2,100000,900000,0",',');
    if (valid) { f[8]="100"; f[20]="100"; }
    f[2]="250"; f[3]="0.249"; f[12]="200000"; f[13]="0";
    f[14]="0.0005267"; f[34]="1000"; f[35]="-1"; f[36]="12000000"; f[41]="200000";
    return String.join(",",f);
  }
  public static void main(String[] args) {
    var manual=new ControlCurrentN20_Processing4(); TestSensingProcessing.graphics(manual);
    manual.background(18); manual.processLine(frame(12)); manual.drawTelemetryPanel();
    manual.g.endDraw(); manual.g.save(args[0]+"/manual-dma.png");
    var pid=new ControlCurrentN20_CurrentPID(); TestSensingProcessing.graphics(pid);
    pid.background(18); pid.processLine(frame(13)); pid.drawLiveValues();
    pid.g.endDraw(); pid.g.save(args[0]+"/pid-dma.png");
    var step=new ControlCurrentN20_StepExperiment(); TestSensingProcessing.graphics(step);
    step.initControls(); step.processLine(frame(12)); step.status="Simulacion visual: sin conexion al motor"; step.draw();
    step.g.endDraw(); step.g.save(args[0]+"/step-dma.png");
  }
}
