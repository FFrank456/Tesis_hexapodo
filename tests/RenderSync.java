import processing.core.PApplet;
class RenderSync {
  static String frame(int status) {
    String[] f=PApplet.split(TestSensingProcessing.data(status==1 ? 1 : 0,status,status==1 ? "0.100" : "nan")+",0,"+(status==1 ? "0.249" : "nan"),',');
    f[2]="250"; f[3]="0.249"; f[13]="0"; f[34]=status==1 ? "8" : "44";
    return String.join(",",f);
  }
  public static void main(String[] args) {
    var manual=new ControlCurrentN20_Processing4(); TestSensingProcessing.graphics(manual);
    manual.background(18); manual.processLine(frame(3)); manual.drawTelemetryPanel();
    manual.g.endDraw(); manual.g.save(args[0]+"/manual-sync.png");
    var pid=new ControlCurrentN20_CurrentPID(); TestSensingProcessing.graphics(pid);
    pid.background(18); pid.processLine(frame(1)); pid.drawLiveValues();
    pid.g.endDraw(); pid.g.save(args[0]+"/pid-sync.png");
    var step=new ControlCurrentN20_StepExperiment(); TestSensingProcessing.graphics(step);
    step.initControls(); step.processLine(frame(3)); step.status="Vista de prueba; sin conexion al motor"; step.draw();
    step.g.endDraw(); step.g.save(args[0]+"/step-sync.png");
  }
}
