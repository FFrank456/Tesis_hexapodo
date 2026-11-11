import processing.awt.PGraphicsJava2D;
class RenderPosition {
  public static void main(String[] args){
    var app=new ControlCurrentN20_CurrentPID();
    var g=new PGraphicsJava2D();g.setParent(app);g.setSize(1366,768);app.g=g;app.width=1366;app.height=768;
    g.beginDraw();app.textFont(app.createFont("Arial",15));app.initControls();app.positionPage=true;app.positionRefSlider.value=1.57f;
    app.advanced=args.length>1;
    app.processLine("ACK,LIMIT,0,40");
    for(int n=0;n<900;n++){
      long t=1000000+n*10000;
      double angle=1.57*(1-Math.exp(-n/120.0));double velocity=1.57/1.2*Math.exp(-n/120.0);
      double reference=velocity>.02 ? velocity : 0;
      int state=1.57-angle<.02 ? 2 : 0;
      app.processLine("POSITION,1,"+t+",1.57,"+(1.57-angle)+","+reference+",6.166,3192,0,2,5,0.02,"+state+",0,"+t+",3");
      app.processLine("SPEED,1,"+t+",1,"+reference+",0,0.02,0.0284,0.01392,0.01,0.1,0,"+t);
      String[] f=TelemetryFixture.fields();f[14]="POSITION";f[24]=""+t;f[50]=""+t;f[52]=""+angle;f[53]=""+velocity;
      f[19]="8.832";f[20]="80090";f[47]=state==2 ? "0" : "0.02";f[15]=f[47];f[18]=state==2 ? "0" : "25";
      app.processLine(String.join(",",f));
    }
    app.lastReply="";app.exportStatus="Vista con datos simulados; no es un ensayo de la planta";
    app.draw();g.endDraw();g.save(args[0]);System.exit(0);
  }
}
