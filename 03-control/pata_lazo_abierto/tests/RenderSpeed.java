import processing.awt.PGraphicsJava2D;
class RenderSpeed {
  public static void main(String[] args) {
    var app=new ControlCurrentN20_CurrentPID();
    int width=args.length>1 ? Integer.parseInt(args[1]) : 1440;
    int height=args.length>2 ? Integer.parseInt(args[2]) : 900;
    var g=new PGraphicsJava2D(); g.setParent(app); g.setSize(width,height);
    app.width=width; app.height=height; app.g=g;
    g.beginDraw(); app.textFont(app.createFont("Arial",15)); app.initControls();
    app.speedPage=args.length<=3 || !args[3].equals("current");
    app.advanced=args.length>4;
    app.processLine("ACK,LIMIT,0.000,40.000");
    for (int n=0;n<1200;n++) {
      app.processLine("SPEED,1,"+(1000000+n*10000)+",1,10,0.1,0.04,0.0284,0.01392,0.037,0.1,0,"+(1000000+n*10000));
      String[] f=TelemetryFixture.fields();
      f[14]=app.speedPage ? "SPEED" : "PID"; f[15]="0.04"; f[16]="0.001"; f[17]="35.5"; f[18]="35.4";
      f[24]=""+(1000000+n*10000);f[50]=f[24];
      f[19]="8.832"; f[20]="80090"; f[22]="35.1"; f[47]=""+(.04*(1-Math.exp(-n/30.0)));
      f[53]=""+(10*(1-Math.exp(-n/30.0)));
      app.processLine(String.join(",",f)); app.samples.get(n).pcTimeMs=n*10;
    }
    app.status="Vista con datos simulados; sin conexion ni accionamiento";
    app.draw(); g.endDraw(); g.save(args[0]); System.exit(0);
  }
}
