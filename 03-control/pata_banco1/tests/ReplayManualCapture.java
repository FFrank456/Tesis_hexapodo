import java.nio.file.*;
class ReplayManualCapture {
  public static void main(String[] args) throws Exception {
    var app=new ControlCurrentN20_Processing4();
    TestSensingProcessing.graphics(app);
    int count=0, absent=0;
    double min=Double.POSITIVE_INFINITY,max=Double.NEGATIVE_INFINITY,sum=0;
    for(String line:Files.readAllLines(Path.of(args[0]))) {
      if(!line.startsWith("DATA,")) continue;
      app.processLine(line);
      if(!app.lastLine.equals(line) || !app.sensing.present) throw new AssertionError("Capture must contain complete DATA v6 frames");
      count++;
      if(!Float.isFinite(app.sensing.adcVoltageMean)) absent++;
      else {
        min=Math.min(min,app.sensing.adcVoltageMean); max=Math.max(max,app.sensing.adcVoltageMean);
        sum+=app.sensing.adcVoltageMean;
      }
    }
    if(count==0) throw new AssertionError("No DATA frames in capture");
    System.out.printf("PASS: %d v6 frames parsed; unavailable mean=%d; CS mean-voltage min/mean/max=%.6f/%.6f/%.6f V%n",count,absent,
      count==absent ? Double.NaN : min,count==absent ? Double.NaN : sum/(count-absent),count==absent ? Double.NaN : max);
    System.out.println("Last: "+app.directionText+" PWM="+app.dutyCw+"/"+app.dutyCcw+
      " status="+app.sensing.status+" "+app.sensing.reason()+" voltage="+app.adcVoltage);
    System.out.println("Rates PWM/attempts/ADC="+app.sensing.pwmHz+"/"+app.sensing.acquisitionHz+"/"+app.sensing.adcHz);
    app.g.endDraw();
  }
}
