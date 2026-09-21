import java.nio.file.*;
class ReplayManualCapture {
  public static void main(String[] args) throws Exception {
    var app=new ControlCurrentN20_Processing4();
    TestSensingProcessing.graphics(app);
    int count=0, absent=0;
    double min=Double.POSITIVE_INFINITY,max=0,sum=0;
    for(String line:Files.readAllLines(Path.of(args[0]))) {
      if(!line.startsWith("DATA,")) continue;
      app.processLine(line);
      if(!app.lastLine.equals(line)) throw new AssertionError("Frame rejected");
      count++;
      if(!Float.isFinite(app.currentRawA)) absent++;
      min=Math.min(min,app.sensing.readUs); max=Math.max(max,app.sensing.readUs); sum+=app.sensing.readUs;
    }
    System.out.printf("PASS: %d real frames parsed; absent current=%d; ADC call min/mean/max=%.0f/%.2f/%.0f us%n",count,absent,min,sum/count,max);
    System.out.println("Last: "+app.directionText+" PWM="+app.dutyCw+"/"+app.dutyCcw+
      " status="+app.sensing.status+" "+app.sensing.reason()+" voltage="+app.adcVoltage);
    System.out.println("Rates PWM/attempts/ADC="+app.sensing.pwmHz+"/"+app.sensing.acquisitionHz+"/"+app.sensing.adcHz);
    app.g.endDraw();
  }
}
