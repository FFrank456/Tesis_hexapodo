import processing.awt.PGraphicsJava2D;
class RenderManualPwm {
  public static void main(String[] args) {
    var app=new ControlCurrentN20_Processing4();
    var g=new PGraphicsJava2D(); g.setParent(app); g.setSize(1100,810);
    app.width=1100; app.height=810; app.g=g;
    g.beginDraw(); app.textFont(app.createFont("Arial",16));
    app.clearButton=app.new Button(32,738,100,34,"Limpiar",0);
    app.pauseButton=app.new Button(144,738,100,34,"Pausar",0);
    for(int i=0;i<200;i++){
      app.sentPwmNormalized=app.normalizePwm(i<50 ? 0 : i<100 ? 50 : i<150 ? -100 : 0);
      app.pushHistory(300,.25f,.10f+(i%8)*.005f);
    }
    app.background(18); app.drawPlot(); app.drawPwmPlot(); app.drawTelemetryPanel();
    g.endDraw(); g.save(args[0]);
  }
}
