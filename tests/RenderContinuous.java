import processing.core.PApplet;
class RenderContinuous {
  static String frame() {
    String[] f=PApplet.split(TestSensingProcessing.data(1,9,"0.080")+",0,0.19920",',');
    f[12]="200000"; f[13]="0"; f[14]="0.0005267"; f[35]="-1"; f[41]="200000";
    return String.join(",",f);
  }
  public static void main(String[] args) {
    var manual=new ControlCurrentN20_Processing4(); TestSensingProcessing.graphics(manual);
    manual.background(18); manual.processLine(frame()); manual.drawTelemetryPanel();
    manual.g.endDraw(); manual.g.save(args[0]+"/manual-continuous.png");
    var pid=new ControlCurrentN20_CurrentPID(); TestSensingProcessing.graphics(pid);
    pid.background(18); pid.processLine(frame()); pid.drawLiveValues();
    pid.g.endDraw(); pid.g.save(args[0]+"/pid-continuous.png");
    var step=new ControlCurrentN20_StepExperiment(); TestSensingProcessing.graphics(step);
    step.initControls(); step.processLine(frame()); step.status="Datos simulados; sin conexion al motor"; step.draw();
    step.g.endDraw(); step.g.save(args[0]+"/step-continuous.png");
  }
}
