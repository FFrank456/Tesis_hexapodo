import processing.serial.*;
import java.io.File;
import java.util.ArrayList;

Serial esp32;

String[] ports = {};
int selectedPort = 0;
boolean serialConnected = false;
String status = "Selecciona puerto y conecta";
String lastLine = "";
String lastCommand = "";
int lastDataMillis = 0;

String direction = "CW";
String controlModeText = "OPEN";

float currentRawA = Float.NaN;
float currentEmaA = Float.NaN;
SensingInfo sensing = new SensingInfo();
float currentRefA = 0.0f;
float currentErrorA = 0.0f;
float controlOutputPercent = 0.0f;
float dutyPercent = 0.0f;
float pidKp = 0.0f;
float pidKi = 8320.0f;
float pidKd = 0.0f;
float pidIntegrator = 0.0f;
float adcVoltage = 0.0f;
float adcRaw = 0.0f;
float pwmFrequencyHz = 0.0f;
float adcSampleRateHz = 0.0f;
float currentEmaAlpha = 0.0f;
int directionCode = 0;
int espTimeMs = 0;
boolean adcCalibrated = false;

ArrayList<Sample> samples = new ArrayList<Sample>();

Button connectButton;
Button refreshButton;
Button modeOpenButton;
Button modePidButton;
Button cwButton;
Button ccwButton;
Button sendPidButton;
Button sendRefButton;
Button stopButton;
Button clearButton;
Button exportButton;
Button scaleAutoButton;
Button scale025Button;
Button scale050Button;
Button scale100Button;
Button scale200Button;

Slider refMilliAmpSlider;
Slider kpSlider;
Slider kiSlider;
Slider kdTimes100Slider;
Slider outputMaxSlider;
Slider manualPwmSlider;
Button applyPwmButton;
Slider activeTextSlider = null;

boolean currentAutoScale = true;
float currentManualScaleA = 1.0f;

void settings() {
  fullScreen();
}

void setup() {
  surface.setTitle("ControlCurrentN20 - Current PID");
  textFont(createFont("Arial", 15));

  connectButton = new Button(898, 44, 112, 34, "Conectar", color(90, 220, 160));
  refreshButton = new Button(1024, 44, 104, 34, "Refrescar", color(120, 160, 220));
  modeOpenButton = new Button(32, 154, 94, 36, "OPEN", color(90, 220, 160));
  modePidButton = new Button(138, 154, 94, 36, "PID", color(80, 180, 255));
  cwButton = new Button(32, 214, 94, 36, "CW", color(80, 180, 255));
  ccwButton = new Button(138, 214, 94, 36, "CCW", color(180, 140, 255));
  sendPidButton = new Button(32, 595, 122, 40, "Enviar PID", color(80, 180, 255));
  sendRefButton = new Button(168, 595, 122, 40, "Enviar ref", color(90, 220, 160));
  stopButton = new Button(304, 595, 92, 40, "STOP", color(235, 82, 82));
  clearButton = new Button(32, 650, 122, 36, "Limpiar", color(130, 150, 180));
  exportButton = new Button(168, 650, 122, 36, "Export CSV", color(250, 190, 90));

  scaleAutoButton = new Button(894, 426, 62, 26, "Auto", color(90, 220, 160));
  scale025Button = new Button(964, 426, 62, 26, "0.25A", color(80, 180, 255));
  scale050Button = new Button(1034, 426, 56, 26, "0.5A", color(80, 180, 255));
  scale100Button = new Button(1098, 426, 46, 26, "1A", color(80, 180, 255));
  scale200Button = new Button(1152, 426, 46, 26, "2A", color(80, 180, 255));

  refMilliAmpSlider = new Slider(32, 315, 350, 0, 1000, "Referencia corriente (mA)", 0);
  kpSlider = new Slider(32, 390, 350, 0, 500, "Kp [%PWM/A]", 3);
  kiSlider = new Slider(32, 465, 350, 0, 12000, "Ki [%PWM/(A*s)]", 2);
  kdTimes100Slider = new Slider(32, 540, 350, 0, 1000, "Kd x100 [%PWM*s/A]", 3);
  outputMaxSlider = new Slider(32, 725, 350, 0, 100, "Limite PWM max (%)", 1);

  refMilliAmpSlider.value = 100;
  kpSlider.value = pidKp;
  kiSlider.value = pidKi;
  kdTimes100Slider.value = pidKd * 100.0f;
  outputMaxSlider.value = 40;
  manualPwmSlider = new Slider(540, 330, 350, -100, 100, "PWM manual (%)", 2);
  manualPwmSlider.value = 0;
  applyPwmButton = new Button(1000, 316, 130, 34, "Aplicar PWM", color(90, 220, 160));
  refreshPorts();
}

void draw() {
  background(18);
  drawHeader();
  drawControls();
  drawLiveValues();
  drawCurrentPlot();
  drawControlPlot();
  drawFooter();
}

void mousePressed() {
  activeTextSlider = null;
  if (applyPwmButton.contains(mouseX, mouseY)) {
    sendCommand("SET," + Float.toString(manualPwmSlider.value));
    return;
  }

  if (connectButton.contains(mouseX, mouseY)) {
    connectSelectedPort();
  } else if (refreshButton.contains(mouseX, mouseY)) {
    refreshPorts();
  } else if (modeOpenButton.contains(mouseX, mouseY)) {
    sendCommand("MODE,OPEN");
  } else if (modePidButton.contains(mouseX, mouseY)) {
    sendCommand("MODE,PID," + direction);
  } else if (cwButton.contains(mouseX, mouseY)) {
    direction = "CW";
    sendCommand("DIR,CW");
  } else if (ccwButton.contains(mouseX, mouseY)) {
    direction = "CCW";
    sendCommand("DIR,CCW");
  } else if (sendPidButton.contains(mouseX, mouseY)) {
    sendPidParameters();
  } else if (sendRefButton.contains(mouseX, mouseY)) {
    sendCurrentReference();
  } else if (stopButton.contains(mouseX, mouseY)) {
    sendCommand("STOP");
  } else if (clearButton.contains(mouseX, mouseY)) {
    clearSamples();
  } else if (exportButton.contains(mouseX, mouseY)) {
    exportData();
  } else if (scaleAutoButton.contains(mouseX, mouseY)) {
    currentAutoScale = true;
  } else if (scale025Button.contains(mouseX, mouseY)) {
    setCurrentScale(0.25f);
  } else if (scale050Button.contains(mouseX, mouseY)) {
    setCurrentScale(0.5f);
  } else if (scale100Button.contains(mouseX, mouseY)) {
    setCurrentScale(1.0f);
  } else if (scale200Button.contains(mouseX, mouseY)) {
    setCurrentScale(2.0f);
  }

  refMilliAmpSlider.mousePressed();
  kpSlider.mousePressed();
  kiSlider.mousePressed();
  kdTimes100Slider.mousePressed();
  outputMaxSlider.mousePressed();
  manualPwmSlider.mousePressed();
}

void mouseDragged() {
  refMilliAmpSlider.mouseDragged();
  kpSlider.mouseDragged();
  kiSlider.mouseDragged();
  kdTimes100Slider.mouseDragged();
  outputMaxSlider.mouseDragged();
  manualPwmSlider.mouseDragged();
}

void mouseReleased() {
  refMilliAmpSlider.mouseReleased();
  kpSlider.mouseReleased();
  kiSlider.mouseReleased();
  kdTimes100Slider.mouseReleased();
  outputMaxSlider.mouseReleased();
  manualPwmSlider.mouseReleased();
}

void keyPressed() {
  if (key == ' ') { sendCommand("STOP"); return; }
  if (activeTextSlider != null) {
    activeTextSlider.keyPressed();
    return;
  }

  if (key == ENTER || key == RETURN) {
    connectSelectedPort();
  } else if (key == ' ') {
    sendCommand("STOP");
  } else if (key == 'p' || key == 'P') {
    sendPidParameters();
  } else if (key == 'i' || key == 'I') {
    sendCurrentReference();
  } else if (key == 'm' || key == 'M') {
    sendCommand(controlModeText.equals("PID") ? "MODE,OPEN" : "MODE,PID," + direction);
  } else if (key == 'e' || key == 'E') {
    exportData();
  } else if (key == 'x' || key == 'X') {
    clearSamples();
  } else if (key == 'y' || key == 'Y') {
    cycleCurrentScale();
  }

  if (keyCode == UP && ports.length > 0) {
    selectedPort = max(0, selectedPort - 1);
  } else if (keyCode == DOWN && ports.length > 0) {
    selectedPort = min(ports.length - 1, selectedPort + 1);
  }
}

void serialEvent(Serial port) {
  String rawLine = port.readStringUntil('\n');
  if (rawLine == null) {
    return;
  }

  processLine(rawLine);
}

void processLine(String rawLine) {
  String line = trim(rawLine);
  if (line.length() == 0) {
    return;
  }

  if (line.startsWith("ACK,") || line.startsWith("ERR,")) {
    status = line;
    return;
  }

  if (!line.startsWith("DATA,")) {
    return;
  }

  String[] data = split(line, ',');
  if (data.length < 25) {
    return;
  }

  SensingInfo nextSensing;
  try { nextSensing=new SensingInfo(data); } catch (RuntimeException e) { status="Trama invalida"; return; }
  lastLine = line;
  lastDataMillis = millis();
  espTimeMs = int(float(data[1]));
  adcRaw = float(data[2]);
  adcVoltage = float(data[3]);
  currentRawA = float(data[4]);
  currentEmaA = float(data[5]);
  directionCode = int(float(data[6]));
  pwmFrequencyHz = float(data[11]);
  adcSampleRateHz = float(data[12]);
  currentEmaAlpha = float(data[14]);
  controlModeText = data[16];
  currentRefA = float(data[17]);
  currentErrorA = float(data[18]);
  controlOutputPercent = float(data[19]);
  dutyPercent = float(data[20]);
  pidKp = float(data[21]);
  pidKi = float(data[22]);
  pidKd = float(data[23]);
  pidIntegrator = float(data[24]);
  adcCalibrated = data.length > 25 && int(float(data[25])) == 1;

  sensing=nextSensing;
  if (!sensing.usable(currentRawA,currentEmaA)) { currentRawA=currentEmaA=currentErrorA=Float.NaN; sensing.valid=false; }
  addSample(new Sample(millis(), espTimeMs, currentRawA, currentEmaA, currentRefA,
                       currentErrorA, controlOutputPercent, dutyPercent, sensing));
}

void refreshPorts() {
  ports = Serial.list();
  selectedPort = constrain(selectedPort, 0, max(0, ports.length - 1));
  status = ports.length == 0 ? "No hay puertos seriales" : "Puerto actualizado";
}

void connectSelectedPort() {
  if (ports.length == 0) {
    status = "No hay puerto disponible";
    serialConnected = false;
    return;
  }

  if (esp32 != null) {
    esp32.stop();
    esp32 = null;
  }

  esp32 = new Serial(this, ports[selectedPort], 460800);
  esp32.clear();
  esp32.bufferUntil('\n');
  serialConnected = true;
  lastDataMillis = millis();
  status = "Conectado a " + ports[selectedPort];
}

void sendPidParameters() {
  float kp = kpSlider.value;
  float ki = kiSlider.value;
  float kd = kdTimes100Slider.value / 100.0f;
  float limitMax = outputMaxSlider.value;
  sendCommand("PID," + nf(kp, 0, 4) + "," + nf(ki, 0, 4) + "," + nf(kd, 0, 4));
  sendCommand("LIMIT,0," + nf(limitMax, 0, 3));
}

void sendCurrentReference() {
  float iref = refMilliAmpSlider.value / 1000.0f;
  sendCommand("IREF," + nf(iref, 0, 4));
  sendCommand("MODE,PID," + direction);
}

void sendCommand(String command) {
  lastCommand = command;
  if (serialConnected && esp32 != null) {
    esp32.write(command + "\n");
  } else {
    status = "Conecta el puerto serial antes de enviar";
  }
}

void addSample(Sample sample) {
  synchronized (samples) {
    samples.add(sample);
    if (samples.size() > 1200) {
      samples.remove(0);
    }
  }
}

void clearSamples() {
  synchronized (samples) {
    samples.clear();
  }
}

ArrayList<Sample> sampleSnapshot() {
  synchronized (samples) {
    return new ArrayList<Sample>(samples);
  }
}

void setCurrentScale(float maxA) {
  currentAutoScale = false;
  currentManualScaleA = maxA;
}

void cycleCurrentScale() {
  if (currentAutoScale) {
    setCurrentScale(0.25f);
  } else if (currentManualScaleA <= 0.25f) {
    setCurrentScale(0.5f);
  } else if (currentManualScaleA <= 0.5f) {
    setCurrentScale(1.0f);
  } else if (currentManualScaleA <= 1.0f) {
    setCurrentScale(2.0f);
  } else {
    currentAutoScale = true;
  }
}

float currentAxisMax(ArrayList<Sample> sourceSamples) {
  if (!currentAutoScale) {
    return currentManualScaleA;
  }

  float maxA = 0.25f;
  for (Sample sample : sourceSamples) {
    if (Float.isFinite(sample.currentEmaA)) maxA=max(maxA,sample.currentEmaA);
    float primary=sample.sensing.primaryCurrent(sample.currentRawA);
    if (Float.isFinite(primary)) maxA=max(maxA,primary);
    if (Float.isFinite(sample.currentRefA)) maxA=max(maxA,sample.currentRefA);
  }
  return ceil(maxA * 10.0f + 1.0f) / 10.0f;
}

void exportData() {
  ArrayList<Sample> exportSamples = sampleSnapshot();
  if (exportSamples.size() == 0) {
    status = "No hay muestras para exportar";
    return;
  }

  Table table = sampleTable(exportSamples);

  String filename = "current_pid_" + year() + nf(month(), 2) + nf(day(), 2) + "_" +
                    nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".csv";
  File output = new File(sketchPath("exports"), filename);
  output.getParentFile().mkdirs();
  saveTable(table, output.getAbsolutePath());
  status = "Exportado: " + output.getAbsolutePath();
}

Table sampleTable(ArrayList<Sample> exportSamples) {
  Table table = new Table();
  table.addColumn("pc_time_ms");
  table.addColumn("esp_time_ms");
  table.addColumn("current_raw_a");
  table.addColumn("current_ema_a");
  table.addColumn("current_ref_a");
  table.addColumn("current_error_a");
  table.addColumn("control_output_percent");
  table.addColumn("duty_percent");
  table.addColumn("kp");
  table.addColumn("ki");
  table.addColumn("kd");
  table.addColumn("pwm_limit_max_percent");
  new SensingInfo().addColumns(table);

  for (Sample sample : exportSamples) {
    TableRow row = table.addRow();
    row.setInt("pc_time_ms", sample.pcTimeMs);
    row.setInt("esp_time_ms", sample.espTimeMs);
    row.setString("current_raw_a", finiteCell(sample.currentRawA));
    row.setString("current_ema_a", finiteCell(sample.currentEmaA));
    row.setFloat("current_ref_a", sample.currentRefA);
    row.setString("current_error_a", finiteCell(sample.currentErrorA));
    row.setFloat("control_output_percent", sample.controlOutputPercent);
    row.setFloat("duty_percent", sample.dutyPercent);
    row.setFloat("kp", pidKp);
    row.setFloat("ki", pidKi);
    row.setFloat("kd", pidKd);
    row.setFloat("pwm_limit_max_percent", outputMaxSlider==null ? Float.NaN : outputMaxSlider.value);
    sample.sensing.writeRow(row);
  }

  return table;
}

void drawHeader() {
  fill(245);
  textSize(24);
  text("Control de corriente PID - N20 + DRV8876", 32, 38);

  int age = millis() - lastDataMillis;
  boolean receiving = serialConnected && age < 500;
  fill(receiving ? color(90, 220, 160) : serialConnected ? color(255, 190, 90) : color(235, 82, 82));
  textSize(14);
  text(receiving ? "recibiendo DATA" : serialConnected ? "conectado, esperando datos" : "sin conexion", 32, 64);

  fill(170);
  text("P: enviar PID   I: enviar ref y PID   M: OPEN/PID   Espacio: STOP   Y: escala   E: exportar", 32, 88);
  drawPortSelector();
}

void drawPortSelector() {
  float x = 704;
  float y = 24;

  fill(225);
  textSize(14);
  text("Puerto serial", x, y);

  if (ports.length == 0) {
    fill(150);
    text("Sin puertos detectados", x, y + 34);
  } else {
    fill(36);
    noStroke();
    rect(x, y + 12, 178, 34, 5);
    fill(230);
    text(selectedPort + ": " + ports[selectedPort], x + 10, y + 35);
  }

  connectButton.draw();
  refreshButton.draw();
}

void drawControls() {
  fill(235);
  textSize(19);
  text("Controlador", 32, 128);

  modeOpenButton.active = controlModeText.equals("OPEN");
  modePidButton.active = controlModeText.equals("PID");
  cwButton.active = direction.equals("CW");
  ccwButton.active = direction.equals("CCW");

  modeOpenButton.draw();
  modePidButton.draw();
  cwButton.draw();
  ccwButton.draw();

  fill(185);
  textSize(14);
  text("Modo: " + controlModeText, 260, 178);
  text("Direccion: " + direction, 260, 238);

  refMilliAmpSlider.draw();
  kpSlider.draw();
  kiSlider.draw();
  kdTimes100Slider.draw();

  sendPidButton.draw();
  sendRefButton.draw();
  stopButton.draw();
  clearButton.draw();
  exportButton.draw();
  outputMaxSlider.draw();
  manualPwmSlider.draw();
  applyPwmButton.draw();
  fill(170); textSize(12); text("+ horario / - antihorario. Enter: guardar; Aplicar PWM: modo OPEN", 540, 374);
}

void drawLiveValues() {
  float x = 540;
  float y = 130;
  float row = 18;

  fill(235);
  textSize(19);
  text("Medicion y esfuerzo", x, y);

  fill(210);
  textSize(15);
  text("I ref: " + nf(currentRefA, 1, 4) + " A", x, y + row);
  text(sensing.version==5 ? "I media DMA: "+currentText(sensing.primaryCurrent(currentRawA)) : "I EMA: "+currentText(currentEmaA), x, y + row * 2);
  if (sensing.version==5) { textSize(12); text("I EMA: "+currentText(currentEmaA),x+290,y+row*4); textSize(15); }
  text("Error: " + currentText(currentErrorA), x, y + row * 3);
  text("u PID: " + nf(controlOutputPercent, 1, 2) + " %", x, y + row * 4);
  text("PWM aplicado: " + nf(dutyPercent, 1, 2) + " %", x, y + row * 5);
  text("Kp/Ki/Kd: " + nf(pidKp, 1, 2) + " / " + nf(pidKi, 1, 2) + " / " + nf(pidKd, 1, 4), x, y + row * 6);
  text("Integrador: " + nf(pidIntegrator, 1, 3) + " %", x, y + row * 7);
  textSize(12);
  text(sensing.reason(), x+290, y+row);
  text(sensing.version==5 ? "ADC medio: "+finiteCell(sensing.adcRawMean)+" | V media: "+finiteCell(sensing.adcVoltageMean) : "ADC: "+(adcRaw>=0 ? nf(adcRaw,0,0) : "sin conversion")+" | V SDK: "+finiteCell(adcVoltage), x+290,y+row*2);
  text(sensing.version>=2 ? sensing.dmaDetails() : "Inicio <= "+finiteCell(sensing.phaseUs)+" us | lectura "+sensing.readUs+" us",x+290,y+row*3);
  text("Nominal: PWM " + rateText(pwmFrequencyHz) + " / ADC " + rateText(adcSampleRateHz) + " Hz", x, y + row * 8);
  text("Observado: PWM " + rateText(sensing.pwmHz) + " / adq. " + rateText(sensing.acquisitionHz)+" / ADC "+rateText(sensing.adcHz)+" Hz", x, y + row * 9);
}

void drawCurrentPlot() {
  ArrayList<Sample> drawSamples = sampleSnapshot();
  float x = 540;
  float y = 390;
  float w = width - x - 34;
  float h = 205;
  float axisMax = currentAxisMax(drawSamples);

  noStroke();
  fill(28);
  rect(x, y, w, h, 6);

  fill(230);
  textSize(16);
  text(sensing.version==5 ? "CS media DMA / EMA; PID pendiente de validacion" : "Corriente: muestra / EMA; PID pendiente de validacion", x + 18, y + 28);
  drawScaleControls();

  float plotX = x + 64;
  float plotY = y + 58;
  float plotW = w - 86;
  float plotH = h - 88;
  drawYAxis(plotX, plotY, plotW, plotH, axisMax, "A");
  drawSamplesLine(drawSamples, plotX, plotY, plotW, plotH, axisMax, 0, color(255, 130, 105));
  drawSamplesLine(drawSamples, plotX, plotY, plotW, plotH, axisMax, 1, color(90, 180, 255));

  if (sensing.version==5) drawSamplesLine(drawSamples,plotX,plotY,plotW,plotH,axisMax,4,color(250,210,100));
  fill(255, 130, 105);
  textSize(13);
  text(sensing.version==5 ? "Media DMA" : "I EMA", plotX + 16, y + h - 18);
  if (sensing.version==5) { fill(250,210,100); text("EMA",plotX+175,y+h-18); }
  fill(90, 180, 255);
  text("I ref", plotX + 115, y + h - 18);
}

void drawControlPlot() {
  ArrayList<Sample> drawSamples = sampleSnapshot();
  float x = 540;
  float y = 615;
  float w = width - x - 34;
  float h = 125;

  noStroke();
  fill(28);
  rect(x, y, w, h, 6);

  fill(230);
  textSize(16);
  text("Esfuerzo de control", x + 18, y + 28);

  float plotX = x + 64;
  float plotY = y + 44;
  float plotW = w - 86;
  float plotH = h - 70;
  drawYAxis(plotX, plotY, plotW, plotH, 100.0f, "%");
  drawSamplesLine(drawSamples, plotX, plotY, plotW, plotH, 100.0f, 2, color(90, 220, 160));
  drawSamplesLine(drawSamples, plotX, plotY, plotW, plotH, 100.0f, 3, color(250, 190, 90));

  fill(90, 220, 160);
  textSize(13);
  text("u PID", plotX + 16, y + h - 14);
  fill(250, 190, 90);
  text("PWM real", plotX + 76, y + h - 14);
}

void drawScaleControls() {
  fill(170);
  textSize(12);
  text("Escala I", 826, 444);

  scaleAutoButton.active = currentAutoScale;
  scale025Button.active = !currentAutoScale && abs(currentManualScaleA - 0.25f) < 0.001f;
  scale050Button.active = !currentAutoScale && abs(currentManualScaleA - 0.5f) < 0.001f;
  scale100Button.active = !currentAutoScale && abs(currentManualScaleA - 1.0f) < 0.001f;
  scale200Button.active = !currentAutoScale && abs(currentManualScaleA - 2.0f) < 0.001f;

  scaleAutoButton.draw();
  scale025Button.draw();
  scale050Button.draw();
  scale100Button.draw();
  scale200Button.draw();
}

void drawYAxis(float x, float y, float w, float h, float axisMax, String unit) {
  stroke(70);
  fill(170);
  textSize(12);
  textAlign(RIGHT, CENTER);

  for (int j = 0; j <= 4; j++) {
    float value = map(j, 0, 4, axisMax, unit.equals("%") ? -axisMax : 0);
    float gy = map(value, unit.equals("%") ? -axisMax : 0, axisMax, y + h, y);
    line(x, gy, x + w, gy);
    text(nf(value, 1, unit.equals("%") ? 0 : 2) + " " + unit, x - 8, gy);
  }

  stroke(120);
  line(x, y, x, y + h);
  line(x, y + h, x + w, y + h);
  textAlign(LEFT, BASELINE);
}

void drawSamplesLine(ArrayList<Sample> sourceSamples, float x, float y, float w, float h,
                     float axisMax, int signal, int c) {
  if (sourceSamples.size() < 2) {
    return;
  }

  int t0 = sourceSamples.get(0).pcTimeMs;
  int t1 = sourceSamples.get(sourceSamples.size() - 1).pcTimeMs;
  int span = max(1, t1 - t0);

  stroke(c);
  strokeWeight(2);
  noFill();
  boolean segment=false;
  for (Sample sample : sourceSamples) {
    float value = sample.sensing.version==5 ? sample.sensing.primaryCurrent(sample.currentRawA) : sample.currentEmaA;
    if (signal == 1) {
      value = sample.currentRefA;
    } else if (signal == 2) {
      value = sample.controlOutputPercent;
    } else if (signal == 3) {
      value = sample.dutyPercent;
    } else if (signal == 4) {
      value = sample.currentEmaA;
    }

    if (!Float.isFinite(value)) { if (segment) endShape(); segment=false; continue; }
    if (!segment) { beginShape(); segment=true; }
    float px = map(sample.pcTimeMs - t0, 0, span, x, x + w);
    float py = map(value, (signal == 2 || signal == 3) ? -axisMax : 0, axisMax, y + h, y);
    vertex(constrain(px, x, x + w), constrain(py, y, y + h));
  }
  if (segment) endShape();
  strokeWeight(1);
}

void drawFooter() {
  fill(145);
  textSize(12);
  text("Estado: " + status, 32, height - 42);
  text("Ultimo comando: " + lastCommand, 32, height - 24);
  text("Ultima DATA: " + lastLine, 32, height - 8);
}

class Sample {
  int pcTimeMs;
  int espTimeMs;
  float currentRawA;
  float currentEmaA;
  float currentRefA;
  float currentErrorA;
  float controlOutputPercent;
  float dutyPercent;
  SensingInfo sensing;

  Sample(int pcTimeMs, int espTimeMs, float currentRawA, float currentEmaA,
         float currentRefA, float currentErrorA, float controlOutputPercent,
         float dutyPercent, SensingInfo sensing) {
    this.pcTimeMs = pcTimeMs;
    this.espTimeMs = espTimeMs;
    this.currentRawA = currentRawA;
    this.currentEmaA = currentEmaA;
    this.currentRefA = currentRefA;
    this.currentErrorA = currentErrorA;
    this.controlOutputPercent = controlOutputPercent;
    this.dutyPercent = dutyPercent;
    this.sensing = sensing;
  }
}

class Button {
  float x;
  float y;
  float w;
  float h;
  String label;
  int baseColor;
  boolean active = false;

  Button(float x, float y, float w, float h, String label, int baseColor) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
    this.label = label;
    this.baseColor = baseColor;
  }

  boolean contains(float px, float py) {
    return px >= x && px <= x + w && py >= y && py <= y + h;
  }

  void draw() {
    boolean hover = contains(mouseX, mouseY);
    noStroke();
    fill(active ? baseColor : hover ? lerpColor(color(48), baseColor, 0.45) : color(42));
    rect(x, y, w, h, 6);
    fill(active ? 18 : 235);
    textAlign(CENTER, CENTER);
    textSize(14);
    text(label, x + w / 2, y + h / 2 - 1);
    textAlign(LEFT, BASELINE);
  }
}

class Slider {
  float x;
  float y;
  float w;
  float inputW = 86;
  float inputH = 30;
  float minValue;
  float maxValue;
  float value;
  String label;
  int decimals;
  boolean dragging = false;
  boolean editing = false;
  String inputText = "";

  Slider(float x, float y, float w, float minValue, float maxValue, String label) {
    this(x, y, w, minValue, maxValue, label, 0);
  }

  Slider(float x, float y, float w, float minValue, float maxValue, String label, int decimals) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.minValue = minValue;
    this.maxValue = maxValue;
    this.label = label;
    this.decimals = decimals;
    this.value = minValue;
  }

  void mousePressed() {
    if (overInput(mouseX, mouseY)) {
      editing = true;
      dragging = false;
      inputText = nf(value, 0, decimalsForRange());
      activeTextSlider = this;
      return;
    }

    editing = false;
    if (mouseX >= x && mouseX <= x + w && mouseY >= y - 18 && mouseY <= y + 28) {
      dragging = true;
      updateValue(mouseX);
    }
  }

  void mouseDragged() {
    if (dragging) {
      updateValue(mouseX);
    }
  }

  void mouseReleased() {
    dragging = false;
  }

  void updateValue(float px) {
    float mapped = map(constrain(px, x, x + w), x, x + w, minValue, maxValue);
    value = roundToDecimals(mapped);
  }

  boolean overInput(float px, float py) {
    float ix = x + w + 18;
    float iy = y - 10;
    return px >= ix && px <= ix + inputW && py >= iy && py <= iy + inputH;
  }

  int decimalsForRange() {
    return decimals;
  }

  float roundToDecimals(float rawValue) {
    if (decimals <= 0) {
      return round(rawValue);
    }

    float factor = pow(10, decimals);
    return round(rawValue * factor) / factor;
  }

  void keyPressed() {
    if (!editing) {
      return;
    }

    if (key == ENTER || key == RETURN) {
      commitInput();
      return;
    }
    if (key == ESC) {
      key = 0;
      editing = false;
      activeTextSlider = null;
      return;
    }
    if (key == BACKSPACE) {
      if (inputText.length() > 0) {
        inputText = inputText.substring(0, inputText.length() - 1);
      }
      return;
    }
    if (key == DELETE) {
      inputText = "";
      return;
    }
    if ((key == '-' || key == '+') && inputText.length() == 0) { inputText += key; return; }
    if ((key >= '0' && key <= '9') || key == '.' || key == ',') {
      char next = key == ',' ? '.' : key;
      if (next == '.' && inputText.indexOf('.') >= 0) {
        return;
      }
      inputText += next;
    }
  }

  void commitInput() {
    if (inputText.length() > 0) {
      float parsed = float(inputText);
      if (!Float.isNaN(parsed) && !Float.isInfinite(parsed) && parsed >= minValue && parsed <= maxValue) {
        value = roundToDecimals(constrain(parsed, minValue, maxValue));
      }
    }
    editing = false;
    activeTextSlider = null;
  }

  void draw() {
    fill(225);
    textSize(15);
    text(label, x, y - 24);
    noStroke();
    fill(45);
    rect(x, y, w, 12, 6);
    float knobX = map(value, minValue, maxValue, x, x + w);
    fill(80, 180, 255);
    rect(x, y, knobX - x, 12, 6);
    ellipse(knobX, y + 6, 28, 28);

    float ix = x + w + 18;
    float iy = y - 10;
    noStroke();
    fill(editing ? color(34, 48, 60) : color(36));
    rect(ix, iy, inputW, inputH, 5);
    stroke(editing ? color(80, 180, 255) : color(70));
    noFill();
    rect(ix, iy, inputW, inputH, 5);

    fill(210);
    textSize(14);
    textAlign(RIGHT, CENTER);
    String shown = editing ? inputText : nf(value, 0, decimalsForRange());
    text(shown, ix + inputW - 8, iy + inputH / 2);
    textAlign(LEFT, BASELINE);
  }
}

// DATA[32..41] describes whether the current conversion can be used.
// Legacy frames retain their measurements, but have no validated sensing flag.
float sensingFloat(String value) {
  return value.equalsIgnoreCase("nan") ? Float.NaN : Float.parseFloat(value);
}

String finiteCell(float value) { return Float.isFinite(value) ? Float.toString(value) : ""; }
String currentText(float value) { return Float.isFinite(value) ? nf(value,1,4)+" A" : "sin lectura"; }
String rateText(float value) { return Float.isFinite(value) && value>=0 ? nf(value,0,0) : "--"; }

class SensingInfo {
  boolean present, valid;
  int version;
  long rejectedWindow, rejectedQuality;
  boolean phaseValidated;
  int status=-1, readUs;
  float phaseUs=Float.NaN, pwmHz=Float.NaN, acquisitionHz=Float.NaN, adcHz=Float.NaN;
  long validCount, attempts, missed, dmaOverflows;
  float voltageEma=Float.NaN;
  // v5: media de las 200 conversiones del bloque; los campos legacy siguen siendo la ultima muestra.
  float adcRawMean=Float.NaN, adcVoltageMean=Float.NaN, currentMeanA=Float.NaN;
  int dmaBlockSamples;
  /** Selecciona la media v5 sin sustituir datos invalidos por la ultima muestra. */
  float primaryCurrent(float legacyCurrent) { return version==5 ? (valid ? currentMeanA : Float.NaN) : legacyCurrent; }
  String primaryLabel() { return version==5 ? "I media DMA" : "I cruda"; }
  SensingInfo() { }
  SensingInfo(String[] fields) {
    if (fields.length>32 && fields.length<42) throw new IllegalArgumentException("Diagnostico incompleto");
    present=fields.length>=42;
    if (!present) return;
    valid=Integer.parseInt(fields[32])==1; status=Integer.parseInt(fields[33]);
    readUs=Integer.parseInt(fields[34]); phaseUs=sensingFloat(fields[35]);
    validCount=Long.parseLong(fields[36]); attempts=Long.parseLong(fields[37]); missed=Long.parseLong(fields[38]);
    pwmHz=sensingFloat(fields[39]); acquisitionHz=sensingFloat(fields[40]); adcHz=sensingFloat(fields[41]);
    // Unknown or contradictory states must never turn an absent reading into zero.
    valid=valid && (status==1 || status==9 || status==11 || status==13 || status==14 || status==15);
    if (fields.length>=44) { dmaOverflows=Long.parseLong(fields[42]); voltageEma=sensingFloat(fields[43]); }
    if (fields.length>44 && fields.length<48) throw new IllegalArgumentException("Diagnostico DMA incompleto");
    if (fields.length>=48) {
      version=Integer.parseInt(fields[44]); rejectedWindow=Long.parseLong(fields[45]);
      rejectedQuality=Long.parseLong(fields[46]); phaseValidated=Integer.parseInt(fields[47])==1;
    }
    if ((status==13 || status==14) && version!=2) valid=false;
    if (status==15 && version!=4 && version!=5) valid=false;
    if (version==5) {
      if (fields.length>=52) {
        adcRawMean=sensingFloat(fields[48]); adcVoltageMean=sensingFloat(fields[49]);
        currentMeanA=sensingFloat(fields[50]); dmaBlockSamples=Integer.parseInt(fields[51]);
      }
      valid=valid && (status==9 || status==15) && dmaBlockSamples==200 &&
        Float.isFinite(adcRawMean) && Float.isFinite(adcVoltageMean) && Float.isFinite(currentMeanA);
      if (!valid) adcRawMean=adcVoltageMean=currentMeanA=Float.NaN;
    }
  }
  boolean usable(float raw, float ema) {
    return version==5 ? valid : (!present || valid) && Float.isFinite(raw) && Float.isFinite(ema);
  }
  String reason() {
    if (!present) return "Sin verificar: firmware sin diagnostico";
    if (version==5 && (status==9 || status==15)) return valid ?
      "Media CS de 200 muestras; A equivalentes sin calibrar" : "Media DMA no valida o bloque incompleto";
    switch (status) {
      case 0: return "Esperando lectura";
      case 1: return valid ? "ON validado; pendiente calibrar corriente" : "Trama de corriente inconsistente";
      case 15: return "Bajo rango: cero asignado; EMA conservado";
      case 9: return "CS continuo + EMA; A equivalentes sin calibrar";
      case 10: return version>=2 ? "Perdida DMA: datos descartados; EMA retenido" : "Desbordamiento DMA";
      case 11: return "Bajo rango ADC: 0 A asignado; EMA conservado";
      case 12: return "DMA: fase PWM sin validar";
      case 13: return phaseValidated ? "DMA ON + EMA (sin calibrar)" : "DMA: 100% continuo + EMA";
      case 14: return "DMA: bajo rango asignado a 0 A; EMA conservado";
      case 2: return "PWM apagado: corriente no medida";
      case 3: return version>=2 ? "Sin muestras dentro de la ventana ON util" : "Sin ventana ON suficiente";
      case 4: return "ADC en el piso (0): corriente no resuelta";
      case 5: return "ADC saturado";
      case 6: return "Error de lectura ADC";
      case 7: return "Error de calibracion ADC";
      case 8: return "Esperando estabilizacion del PWM";
      default: return "Estado de corriente desconocido";
    }
  }
  /** Resume conteos ADC acumulados; el CSV conserva los enteros completos. */
  String compactCount(long n) { return n>=1000000 ? nf(n/1000000.0f,0,1)+"M" : n>=1000 ? nf(n/1000.0f,0,1)+"k" : Long.toString(n); }
  String dmaDetails() { return (version==5 ? "Bloque "+dmaBlockSamples+" | CS " : version>=3 ? "CS " : "ON ") + compactCount(validCount) + " | fuera " + compactCount(rejectedWindow) + " | invalidas " + compactCount(rejectedQuality); }
  void addColumns(Table table) {
    for (String name:new String[]{"current_valid","current_status","current_status_text","adc_read_us","sample_phase_us",
      "current_valid_count","sample_attempt_count","sample_missed_count","pwm_frequency_observed_hz",
      "acquisition_rate_observed_hz","adc_rate_observed_hz","dma_overflows","adc_voltage_ema","measurement_basis","sensing_version","dma_rejected_window","dma_rejected_quality","dma_phase_validated","raw_zero_assigned","adc_raw_mean","adc_voltage_mean","current_mean_a","dma_block_samples"}) table.addColumn(name);
  }
  void writeRow(TableRow row) {
    row.setString("dma_overflows",Long.toString(dmaOverflows));
    row.setString("adc_voltage_ema",finiteCell(voltageEma));
    row.setString("raw_zero_assigned",(version==4 || version==5) ? (status==15 ? "1" : "0") : "");
    row.setString("sensing_version",Integer.toString(version));
    row.setString("adc_raw_mean",version==5 ? finiteCell(adcRawMean) : "");
    row.setString("adc_voltage_mean",version==5 ? finiteCell(adcVoltageMean) : "");
    row.setString("current_mean_a",version==5 ? finiteCell(currentMeanA) : "");
    row.setString("dma_block_samples",version==5 ? Integer.toString(dmaBlockSamples) : "");
    row.setString("dma_rejected_window",version>=2 ? Long.toString(rejectedWindow) : "");
    row.setString("dma_rejected_quality",version>=2 ? Long.toString(rejectedQuality) : "");
    row.setString("dma_phase_validated",version>=2 ? (phaseValidated ? "1" : "0") : "");
    row.setString("measurement_basis",version==5 ? (valid ? "dma_block_mean_CS_raw_zero_assigned" : "unavailable") : version==4 && (status==9 || status==15) ? "continuous_CS_raw_zero_assigned" : status==13 ? (phaseValidated ? "DMA_ON_samples" : "DMA_DC_100_percent") : status==14 ? "DMA_below_range_zero" : status==9 ? "continuous_CS_all_phases" : status==11 ? "ON_below_range_zero" : status==1 ? "validated_ON_current" : "unavailable");
    row.setString("current_valid",present ? (valid ? "1" : "0") : "");
    row.setString("current_status",Integer.toString(status)); row.setString("current_status_text",reason());
    row.setString("adc_read_us",present ? Integer.toString(readUs) : "");
    row.setString("sample_phase_us",present && phaseUs>=0 ? finiteCell(phaseUs) : "");
    row.setString("current_valid_count",present ? Long.toString(validCount) : "");
    row.setString("sample_attempt_count",present ? Long.toString(attempts) : "");
    row.setString("sample_missed_count",present ? Long.toString(missed) : "");
    row.setString("pwm_frequency_observed_hz",finiteCell(pwmHz));
    row.setString("acquisition_rate_observed_hz",finiteCell(acquisitionHz)); row.setString("adc_rate_observed_hz",finiteCell(adcHz));
  }
}
