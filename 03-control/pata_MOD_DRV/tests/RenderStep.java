import processing.awt.PGraphicsJava2D;
class RenderStep {
  public static void main(String[] args) {
    var app=new ControlCurrentN20_StepExperiment();
    app.width=1180; app.height=760;
    var graphics=new PGraphicsJava2D();
    graphics.setParent(app); graphics.setPrimary(false); graphics.setSize(app.width,app.height);
    app.g=graphics;
    graphics.beginDraw();
    app.textFont(app.createFont("Arial",15));
    app.initControls();
    app.draw();
    graphics.endDraw();
    graphics.save(args[0]);
    System.out.println("Rendered Processing layout");
  }
}
