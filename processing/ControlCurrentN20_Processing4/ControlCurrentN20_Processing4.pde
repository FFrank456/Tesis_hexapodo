import processing.serial.*;

// false: mostrar solo la medida principal (media DMA en v5); no modifica el filtro del firmware.
final boolean SHOW_EMA = true;
// Consigna enviada por esta interfaz; NaN hasta realizar un envio serial.
float sentPwmNormalized = Float.NaN;


Serial esp32;

String[] ports = {};
int selectedPort = 1;
boolean serialConnected = false;
boolean plotPaused = false;

float timeMs = 0;
float adcRaw = 0;
float adcVoltage = 0;
float currentRawA = Float.NaN;
float currentEmaA = Float.NaN;
float currentDisplayA = Float.NaN;
SensingInfo sensing = new SensingInfo();
int directionCode = 0;
String directionText = "STOP";
float dutyCw = 0;
float dutyCcw = 0;
float adcSampleCount = 0;
float pwmFrequencyHz = 0;
float adcSampleRateHz = 0;
float adcSampleTick = 0;
float currentEmaAlpha = 0;
String lastLine = "";
String lastCommand = "";
int lastDataMillis = 0;

float commandDuty = 0;
Slider activeTextSlider = null;
Button applyButton;
String commandDirection = "STOP";

final int HISTORY_SIZE = 420;
float[] pwmHistory = new float[HISTORY_SIZE];
float[] rawHistory = new float[HISTORY_SIZE];
float[] voltageHistory = new float[HISTORY_SIZE];
float[] currentRawHistory = new float[HISTORY_SIZE];
float[] currentEmaHistory = new float[HISTORY_SIZE];
int historyIndex = 0;
boolean historyFilled = false;
float currentAxisMaxA = 1.5;

Button stopButton;
Button cwButton;
Button ccwButton;
Button connectButton;
Button refreshButton;
Button clearButton;
Button pauseButton;
Slider dutySlider;

/** Inicializa ventana, controles y lista de puertos; no acciona el motor. */
void setup() {
  size(1100, 810);
  surface.setTitle("ControlCurrentN20 - Processing 4");
  textFont(createFont("Arial", 16));

  stopButton = new Button(32, 180, 120, 42, "STOP", color(235, 82, 82));
  cwButton = new Button(166, 180, 120, 42, "Horario", color(80, 180, 255));
  ccwButton = new Button(300, 180, 140, 42, "Antihorario", color(180, 140, 255));
  connectButton = new Button(838, 46, 112, 34, "Conectar", color(90, 220, 160));
  refreshButton = new Button(962, 46, 96, 34, "Refrescar", color(120, 160, 220));
  clearButton = new Button(32, 738, 100, 34, "Limpiar", color(120, 160, 220));
  pauseButton = new Button(144, 738, 100, 34, "Pausar", color(250, 190, 90));
  dutySlider = new Slider(32, 265, 290, -100, 100, "PWM % (+ horario / - antihorario)", 2);

  dutySlider.value = 0;
  applyButton = new Button(32, 294, 120, 30, "Aplicar PWM", color(90, 220, 160));
  refreshPorts();
}

/** Redibuja controles, telemetria e historiales en cada fotograma. */
void draw() {
  background(18);
  drawHeader();
  drawControlPanel();
  drawTelemetryPanel();
  drawPlot();
  drawPwmPlot();
  drawFooter();
}

/** Distribuye el clic entre botones, deslizador y campo de entrada. */
void mousePressed() {
  activeTextSlider = null;
  if (applyButton.contains(mouseX, mouseY)) { applyManualPWM(); return; }
  if (connectButton.contains(mouseX, mouseY)) {
    connectSelectedPort();
  } else if (refreshButton.contains(mouseX, mouseY)) {
    refreshPorts();
  } else if (stopButton.contains(mouseX, mouseY)) {
    sendMotorCommand("STOP");
  } else if (cwButton.contains(mouseX, mouseY)) {
    sendMotorCommand("CW");
  } else if (ccwButton.contains(mouseX, mouseY)) {
    sendMotorCommand("CCW");
  } else if (clearButton.contains(mouseX, mouseY)) {
    clearHistory();
  } else if (pauseButton.contains(mouseX, mouseY)) {
    plotPaused = !plotPaused;
    pauseButton.label = plotPaused ? "Continuar" : "Pausar";
  }

  dutySlider.mousePressed();
}

/** Actualiza el deslizador mientras se arrastra. */
void mouseDragged() {
  dutySlider.mouseDragged();
  commandDuty = dutySlider.value;
}

/** Finaliza el arrastre y procesa el cambio manual. */
void mouseReleased() {
  if (dutySlider.dragging) {
    dutySlider.mouseReleased();
    commandDuty = dutySlider.value;
    applyManualPWM();
  }
}

/** Procesa teclado y edicion numerica sin llamadas recursivas. */
void keyPressed() {
  if (key == ' ') { sendMotorCommand("STOP"); return; }
  if (activeTextSlider != null) {
    activeTextSlider.keyPressed();
    return;
  }
  if (key == 'r' || key == 'R') {
    refreshPorts();
  } else if (key == ENTER || key == RETURN) {
    connectSelectedPort();
  } else if (key == ' ') {
    sendMotorCommand("STOP");
  } else if (key == 'h' || key == 'H' || key == 'c' || key == 'C') {
    sendMotorCommand("CW");
  } else if (key == 'a' || key == 'A') {
    sendMotorCommand("CCW");
  } else if (key == 'x' || key == 'X') {
    clearHistory();
  } else if (key == 'p' || key == 'P') {
    plotPaused = !plotPaused;
    pauseButton.label = plotPaused ? "Continuar" : "Pausar";
  } else if (key == '+' || key == '=') {
    setCommandDuty(commandDuty + 5);
  } else if (key == '-' || key == '_') {
    setCommandDuty(commandDuty - 5);
  }

  if (keyCode == UP && ports.length > 0) {
    selectedPort = max(0, selectedPort - 1);
  } else if (keyCode == DOWN && ports.length > 0) {
    selectedPort = min(ports.length - 1, selectedPort + 1);
  } else if (keyCode == RIGHT) {
    setCommandDuty(commandDuty + 1);
  } else if (keyCode == LEFT) {
    setCommandDuty(commandDuty - 1);
  }
}

/** Recibe una linea serial completa y la entrega al decodificador. */
void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');
  if (line == null) {
    return;
  }

  processLine(line);
}

/** Decodifica DATA, comprueba validez y registra una muestra si no hay pausa. */
void processLine(String line) {
  line = trim(line);
  if (!line.startsWith("DATA,")) {
    return;
  }

  String[] data = split(line, ',');
  if (data.length < 9) {
    return;
  }

  SensingInfo nextSensing;
  try { nextSensing = new SensingInfo(data); } catch (RuntimeException e) { return; }
  lastLine = line;
  lastDataMillis = millis();
  timeMs = float(data[1]);
  adcRaw = float(data[2]);
  adcVoltage = float(data[3]);
  currentRawA = float(data[4]);

  if (data.length >= 14) {
    currentEmaA = float(data[5]);
    directionCode = int(data[6]);
    directionText = data[7];
    dutyCw = float(data[8]);
    dutyCcw = float(data[9]);
    adcSampleCount = float(data[10]);
    pwmFrequencyHz = float(data[11]);
    adcSampleRateHz = float(data[12]);
    adcSampleTick = float(data[13]);

    if (data.length >= 15) {
      currentEmaAlpha = float(data[14]);
    }
  } else {
    currentEmaA = currentRawA;
    directionCode = int(data[5]);
    directionText = data[6];
    dutyCw = float(data[7]);
    dutyCcw = float(data[8]);
  }

  sensing=nextSensing;
  if (!sensing.usable(currentRawA,SHOW_EMA ? currentEmaA : currentRawA)) { currentRawA=currentEmaA=Float.NaN; sensing.valid=false; }
  currentDisplayA=sensing.primaryCurrent(currentRawA);
  updateCurrentAxis();

  if (!plotPaused) {
    pushHistory(adcRaw, adcVoltage, currentDisplayA, currentEmaA);
  }
}

/** Enumera los puertos seriales y limita el indice seleccionado. */
void refreshPorts() {
  ports = Serial.list();
  selectedPort = constrain(selectedPort, 0, max(0, ports.length - 1));
}

/** Cierra la conexion anterior y abre el puerto elegido a 460800 baudios. */
void connectSelectedPort() {
  if (ports.length == 0) {
    serialConnected = false;
    return;
  }

  if (esp32 != null) {
    esp32.stop();
    esp32 = null;
  }

  esp32 = new Serial(this, ports[selectedPort], 460800);
  sentPwmNormalized = Float.NaN;
  esp32.clear();
  esp32.bufferUntil('\n');
  serialConnected = true;
  lastDataMillis = millis();
}

/** Ajusta signo o cero de la consigna y envia el comando manual. */
void sendMotorCommand(String direction) {
  dutySlider.editing = false;
  activeTextSlider = null;
  dutySlider.value = direction.equals("STOP") ? 0 :
                     direction.equals("CW") ? abs(dutySlider.value) : -abs(dutySlider.value);
  applyManualPWM();
}

/** Envia SET y registra su magnitud solo despues de escribir al puerto. */
void applyManualPWM() {
  commandDuty = dutySlider.value;
  commandDirection = commandDuty > 0 ? "CW" : commandDuty < 0 ? "CCW" : "STOP";
  lastCommand = "SET," + Float.toString(commandDuty);
  if (esp32 != null && serialConnected) {
    esp32.write(lastCommand + "\n");
    sentPwmNormalized = normalizePwm(commandDuty);
  }
}

/** Limita la consigna a -100..100 por ciento antes de enviarla. */
void setCommandDuty(float duty) {
  dutySlider.value = constrain(duty, -100, 100);
  applyManualPWM();
}

/** Inserta corriente, voltaje y PWM enviado en el mismo buffer circular. */
void pushHistory(float raw, float voltage, float currentRaw, float currentEma) {
  rawHistory[historyIndex] = raw;
  voltageHistory[historyIndex] = voltage;
  currentRawHistory[historyIndex] = currentRaw;
  currentEmaHistory[historyIndex] = currentEma;
  pwmHistory[historyIndex] = sentPwmNormalized;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  if (historyIndex == 0) {
    historyFilled = true;
  }
}

/** Vacia los historiales sin cambiar el PWM del motor. */
void clearHistory() {
  historyIndex = 0;
  historyFilled = false;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    rawHistory[i] = 0;
    voltageHistory[i] = 0;
    currentRawHistory[i] = 0;
    currentEmaHistory[i] = 0;
    pwmHistory[i] = Float.NaN;
  }
}

/** Ajusta la escala de corriente segun las curvas visibles y finitas. */
void updateCurrentAxis() {
  if (!Float.isFinite(currentDisplayA)) return;
  float measuredMax = SHOW_EMA && Float.isFinite(currentEmaA) ? max(currentDisplayA, currentEmaA) : currentDisplayA;
  float targetMax = max(0.5, ceil(measuredMax * 5.0 + 1.0) / 5.0);
  currentAxisMaxA = max(currentAxisMaxA * 0.995, targetMax);
}

/** Dibuja titulo, estado de conexion y ayuda de teclado. */
void drawHeader() {
  fill(245);
  textSize(25);
  text("ControlCurrentN20", 32, 38);

  int age = millis() - lastDataMillis;
  boolean receiving = serialConnected && age < 800;
  fill(receiving ? color(90, 220, 160) : serialConnected ? color(255, 190, 90) : color(235, 82, 82));
  String status = receiving ? "recibiendo datos" : serialConnected ? "conectado, esperando DATA" : "sin conexion";
  textSize(14);
  text(status, 32, 64);

  fill(170);
  text("Espacio: STOP   H/C: horario   A: antihorario   +/-: duty rapido   flechas: puerto/duty   P: pausa   X: limpiar", 32, 88);

  drawPortSelector();
}

/** Dibuja selector de puerto y botones de conexion. */
void drawPortSelector() {
  float x = 650;
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
    rect(x, y + 12, 172, 34, 5);
    fill(230);
    text(selectedPort + ": " + ports[selectedPort], x + 10, y + 35);
  }

  connectButton.draw();
  refreshButton.draw();
}

/** Presenta controles de direccion, PWM y ultima consigna. */
void drawControlPanel() {
  fill(235);
  textSize(19);
  text("Control manual", 32, 142);

  stopButton.active = commandDirection.equals("STOP");
  cwButton.active = commandDirection.equals("CW");
  ccwButton.active = commandDirection.equals("CCW");
  stopButton.draw();
  cwButton.draw();
  ccwButton.draw();

  dutySlider.draw();
  applyButton.draw();
  fill(170); textSize(12); text("Enter: guardar valor; Aplicar PWM: enviar", 166, 314);

  fill(185);
  textSize(14);
  text("Comando: " + (lastCommand.length() == 0 ? "-" : lastCommand), 32, 334);
  text("PWM objetivo: " + nf(commandDuty, 1, 2) + " %", 32, 358);
}

/** Presenta medidas, validez del sensado y tiempos de adquisicion. */
void drawTelemetryPanel() {
  float x = 520;
  float y = 132;
  float row = 22;

  fill(235);
  textSize(19);
  text("Telemetria ESP32", x, y);

  textSize(13);
  fill(210);
  text("Tiempo: " + nf(timeMs / 1000.0, 1, 2) + " s", x, y + row);
  text(sensing.version==5 ? "ADC medio: "+finiteCell(sensing.adcRawMean) : "ADC: " + (adcRaw>=0 ? nf(adcRaw,0,0)+" / 4095" : "sin conversion"), x, y + row * 2);
  text(sensing.version==5 ? "V media: "+finiteCell(sensing.adcVoltageMean)+" | SDK ultimo: "+finiteCell(adcVoltage) : "V SDK: " + finiteCell(adcVoltage) + " V" + (SHOW_EMA ? " | EMA: " + (Float.isFinite(sensing.voltageEma) ? nf(sensing.voltageEma,1,3)+" V" : "--") : ""), x, y + row * 3);
  text(sensing.primaryLabel()+": " + currentText(currentDisplayA), x, y + row * 4);
  text(SHOW_EMA ? "I EMA: " + currentText(currentEmaA) : "EMA oculto; medida principal sin cambios", x, y + row * 5);
  text(sensing.reason(), x, y + row * 6);
  text("Direccion real: " + directionText, x, y + row * 7);
  text("PWM CW / CCW: " + nf(dutyCw, 0, 2) + " / " + nf(-dutyCcw,0,2) + " %", x, y + row * 8);
  text("Nominal: PWM " + rateText(pwmFrequencyHz) + " / ADC " + rateText(adcSampleRateHz) + " Hz", x, y + row * 9);
  text("Obs. ADC " + rateText(sensing.adcHz) + " | consulta " + rateText(sensing.acquisitionHz) + " Hz", x, y + row * 10);
  text(sensing.version>=2 ? sensing.dmaDetails() : "ADC " + rateText(sensing.adcHz) + " Hz | inicio <= " + finiteCell(sensing.phaseUs) + " us | lectura " + sensing.readUs + " us", x, y + row * 11);

  drawBar(sensing.version==5 ? "ADC medio" : "ADC", sensing.version==5 ? sensing.adcRawMean : adcRaw, 0, 4095, 860, y + 23, 180, 18, color(80, 180, 255));
  drawBar(sensing.version==5 ? "V media DMA" : "Voltaje", sensing.version==5 ? sensing.adcVoltageMean : adcVoltage, 0, 3.3, 860, y + 79, 180, 18, color(90, 220, 160));
  drawBar(sensing.primaryLabel(), currentDisplayA, 0, currentAxisMaxA, 860, y + 135, 180, 18, color(250, 210, 100));
  if (SHOW_EMA) drawBar("I EMA", currentEmaA, 0, currentAxisMaxA, 860, y + 163, 180, 18, color(255, 130, 105));
  drawBar("Duty CW", dutyCw, 0, 100, 860, y + 191, 180, 18, color(255, 130, 105));
}

/** Dibuja una barra limitada al rango; conserva las lecturas ausentes. */
void drawBar(String label, float value, float minValue, float maxValue, float x, float y, float w, float h, int c) {
  fill(185);
  textSize(12);
  text(label, x, y - 6);

  noStroke();
  fill(43);
  rect(x, y, w, h, 4);

  if (!Float.isFinite(value) || value<minValue) return;
  fill(c);
  float barW = constrain(map(value, minValue, maxValue, 0, w), 0, w);
  rect(x, y, barW, h, 4);
}

/** Dibuja la media de bloque v5 (ultima muestra en legacy) y opcionalmente EMA. */
void drawPlot() {
  float x = 32;
  float y = 395;
  float w = width - 64;
  float h = 190;

  noStroke();
  fill(28);
  rect(x, y, w, h, 6);

  drawCurrentAxis(x, y, w, h);
  drawHistoryLine(currentRawHistory, 0, currentAxisMaxA, x + 58, y + 16, w - 72, h - 36, color(250, 210, 100));
  if (SHOW_EMA) drawHistoryLine(currentEmaHistory, 0, currentAxisMaxA, x + 58, y + 16, w - 72, h - 36, color(255, 130, 105));

  fill(220);
  textSize(14);
  text(sensing.version>=3 ? "CS (A equivalentes)" : sensing.version>=2 ? "I muestra DMA (A)" : sensing.status==9 ? "CS continuo (A eq.)" : "Corriente ON (A)", x + 70, y + 24);
  fill(250, 210, 100);
  text(sensing.version==5 ? "media DMA" : "cruda", x + 245, y + 24);
  fill(255, 130, 105);
  if (SHOW_EMA) text("EMA alpha/muestra=" + nf(currentEmaAlpha, 1, 6), x + 345, y + 24);

  clearButton.draw();
  pauseButton.draw();
}

/** Convierte porcentaje firmado a magnitud 0..1; el sentido se muestra aparte. */
float normalizePwm(float percent) {
  return Float.isFinite(percent) ? constrain(abs(percent)/100.0,0,1) : Float.NaN;
}

/** Dibuja la consigna enviada como escalones; no representa pulsos de 10 kHz
 * ni confirma que el ESP32 haya aplicado el comando. Comparte muestras/pausa
 * con corriente; antes del primer envio no se inventan valores. */
void drawPwmPlot() {
  float x=90, y=630, w=width-136, h=72;
  noStroke(); fill(28); rect(32,598,width-64,126,6);
  fill(210); textSize(13); text("PWM enviado |magnitud| 0..1 (sentido: "+commandDirection+")",102,618);
  textAlign(RIGHT,CENTER);
  for (int i=0;i<=2;i++) {
    float value=i/2.0, gy=y+h-value*h;
    stroke(65); line(x,gy,x+w,gy); fill(180); text(nf(value,1,1),x-8,gy);
  }
  textAlign(LEFT,BASELINE);
  int count=historyFilled ? HISTORY_SIZE : historyIndex;
  stroke(80,180,255); strokeWeight(2);
  for (int i=1;i<count;i++) {
    int a=historyFilled ? (historyIndex+i-1)%HISTORY_SIZE : i-1;
    int b=historyFilled ? (historyIndex+i)%HISTORY_SIZE : i;
    if (!Float.isFinite(pwmHistory[a]) || !Float.isFinite(pwmHistory[b])) continue;
    float x0=map(i-1,0,count-1,x,x+w), x1=map(i,0,count-1,x,x+w);
    float y0=y+h*(1-pwmHistory[a]), y1=y+h*(1-pwmHistory[b]);
    line(x0,y0,x1,y0); line(x1,y0,x1,y1);
  }
  strokeWeight(1);
}

/** Dibuja divisiones y etiquetas del eje de corriente en amperios. */
void drawCurrentAxis(float x, float y, float w, float h) {
  float plotX = x + 58;
  float plotY = y + 16;
  float plotW = w - 72;
  float plotH = h - 36;

  stroke(70);
  fill(170);
  textSize(12);
  textAlign(RIGHT, CENTER);

  for (int i = 0; i <= 5; i++) {
    float value = map(i, 0, 5, currentAxisMaxA, 0);
    float gy = map(value, 0, currentAxisMaxA, plotY + plotH, plotY);
    line(plotX, gy, plotX + plotW, gy);
    text(nf(value, 1, 2) + " A", plotX - 8, gy);
  }

  stroke(120);
  line(plotX, plotY, plotX, plotY + plotH);
  line(plotX, plotY + plotH, plotX + plotW, plotY + plotH);
  textAlign(LEFT, BASELINE);
}

/** Recorre el buffer circular en orden temporal y corta curvas en NaN. */
void drawHistoryLine(float[] values, float minValue, float maxValue, float x, float y, float w, float h, int c) {
  int count = historyFilled ? HISTORY_SIZE : historyIndex;
  if (count < 2) {
    return;
  }

  stroke(c);
  strokeWeight(2);
  noFill();
  boolean segment=false;
  for (int i = 0; i < count; i++) {
    int sourceIndex = historyFilled ? (historyIndex + i) % HISTORY_SIZE : i;
    if (!Float.isFinite(values[sourceIndex])) { if (segment) endShape(); segment=false; continue; }
    if (!segment) { beginShape(); segment=true; }
    float px = map(i, 0, count - 1, x, x + w);
    float py = map(values[sourceIndex], minValue, maxValue, y + h, y);
    vertex(px, constrain(py, y, y + h));
  }
  if (segment) endShape();
  strokeWeight(1);
}

/** Muestra la ultima linea recibida para diagnostico. */
void drawFooter() {
  fill(145);
  textSize(12);
  text("Ultima linea: " + lastLine, 32, height - 18);
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

  /** Comprueba si un punto queda dentro de los limites del boton. */
  boolean contains(float px, float py) {
    return px >= x && px <= x + w && py >= y && py <= y + h;
  }

  /** Redibuja controles, telemetria e historiales en cada fotograma. */
  void draw() {
    boolean hover = contains(mouseX, mouseY);
    noStroke();
    fill(active ? baseColor : hover ? lerpColor(color(48), baseColor, 0.45) : color(42));
    rect(x, y, w, h, 6);

    fill(active ? 18 : 235);
    textAlign(CENTER, CENTER);
    textSize(15);
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

  /** Distribuye el clic entre botones, deslizador y campo de entrada. */
  void mousePressed() {
  activeTextSlider = null;
  if (applyButton.contains(mouseX, mouseY)) { applyManualPWM(); return; }
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

  /** Actualiza el deslizador mientras se arrastra. */
  void mouseDragged() {
    if (dragging) {
      updateValue(mouseX);
    }
  }

  /** Finaliza el arrastre y procesa el cambio manual. */
  void mouseReleased() {
    dragging = false;
  }

  /** Convierte la posicion del raton a un valor limitado del deslizador. */
  void updateValue(float px) {
    float mapped = map(constrain(px, x, x + w), x, x + w, minValue, maxValue);
    value = roundToDecimals(mapped);
  }

  /** Comprueba si el punto pertenece al cuadro de edicion numerica. */
  boolean overInput(float px, float py) {
    float ix = x + w + 18;
    float iy = y - 10;
    return px >= ix && px <= ix + inputW && py >= iy && py <= iy + inputH;
  }

  int decimalsForRange() {
    return decimals;
  }

  /** Redondea el valor al numero de decimales configurado. */
  float roundToDecimals(float rawValue) {
    if (decimals <= 0) {
      return round(rawValue);
    }

    float factor = pow(10, decimals);
    return round(rawValue * factor) / factor;
  }

  /** Procesa teclado y edicion numerica sin llamadas recursivas. */
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

  /** Valida el texto numerico y confirma la edicion dentro del rango. */
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

  /** Redibuja controles, telemetria e historiales en cada fotograma. */
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
/** Convierte un campo de telemetria y admite valores NaN. */
float sensingFloat(String value) {
  return value.equalsIgnoreCase("nan") ? Float.NaN : Float.parseFloat(value);
}

/** Formatea un numero finito o una celda vacia para un dato ausente. */
String finiteCell(float value) { return Float.isFinite(value) ? Float.toString(value) : ""; }
/** Formatea amperios o indica ausencia de lectura. */
String currentText(float value) { return Float.isFinite(value) ? nf(value,1,4)+" A" : "sin lectura"; }
/** Formatea una frecuencia valida o un marcador de ausencia. */
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
  /** Comprueba validez del protocolo y existencia de las medidas. */
  boolean usable(float raw, float ema) {
    return version==5 ? valid : (!present || valid) && Float.isFinite(raw) && Float.isFinite(ema);
  }
  /** Traduce el estado numerico de sensado a un mensaje legible. */
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
  /** Diagnostico DMA: conteos acumulados desde el arranque; no son tramas UART. */
  String compactCount(long n) { return n>=1000000 ? nf(n/1000000.0f,0,1)+"M" : n>=1000 ? nf(n/1000.0f,0,1)+"k" : Long.toString(n); }
  String dmaDetails() {
    return (version==5 ? "Bloque "+dmaBlockSamples+" | CS " : version>=3 ? "CS " : "ON ") + compactCount(validCount) + " | fuera " + compactCount(rejectedWindow) + " | invalidas " + compactCount(rejectedQuality);
  }
  /** Agrega columnas de diagnostico de corriente a una tabla. */
  void addColumns(Table table) {
    for (String name:new String[]{"current_valid","current_status","current_status_text","adc_read_us","sample_phase_us",
      "current_valid_count","sample_attempt_count","sample_missed_count","pwm_frequency_observed_hz",
      "acquisition_rate_observed_hz","adc_rate_observed_hz","dma_overflows","adc_voltage_ema","measurement_basis","sensing_version","dma_rejected_window","dma_rejected_quality","dma_phase_validated","raw_zero_assigned","adc_raw_mean","adc_voltage_mean","current_mean_a","dma_block_samples"}) table.addColumn(name);
  }
  /** Exporta diagnosticos conservando validez y significado de la medida. */
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
