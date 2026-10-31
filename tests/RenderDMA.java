import processing.core.PApplet;
class RenderDMA {
  static String frame(int status) {
    boolean valid=status==9;
    String[] f=PApplet.split(TestSensingProcessing.data(valid ? 1 : 0,status,valid ? "0.100" : "nan"),',');
    f[40]=valid ? "0" : "1";
    if (!valid) { f[45]="nan"; f[46]="nan"; f[48]="0"; }
    return String.join(",",f);
  }
  public static void main(String[] args) {
    var manual=new ControlCurrentN20_Processing4(); TestSensingProcessing.graphics(manual);
    manual.background(18); manual.processLine(frame(10)); manual.drawTelemetryPanel();
    manual.g.endDraw(); manual.g.save(args[0]+"/manual-dma.png");
    var pid=new ControlCurrentN20_CurrentPID(); TestSensingProcessing.graphics(pid);
    pid.background(18); pid.processLine(frame(9)); pid.drawLiveValues();
    pid.g.endDraw(); pid.g.save(args[0]+"/pid-dma.png");
    var step=new ControlCurrentN20_StepExperiment(); TestSensingProcessing.graphics(step);
    step.initControls(); step.processLine(frame(10)); step.status="Simulacion visual: sin conexion al motor"; step.draw();
    step.g.endDraw(); step.g.save(args[0]+"/step-dma.png");
  }
}
