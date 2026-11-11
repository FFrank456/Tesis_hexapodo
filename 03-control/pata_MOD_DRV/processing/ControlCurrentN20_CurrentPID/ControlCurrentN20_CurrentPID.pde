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
float currentMeanA = Float.NaN;
SensingInfo sensing = new SensingInfo();
EncoderInfo encoder = new EncoderInfo();
float currentRefA = 0.0f;
float currentErrorA = 0.0f;
float controlOutputPercent = 0.0f;
float dutyPercent = 0.0f;
float pwmLimitConfirmed=Float.NaN;
float pidKp = 8.832f;
float pidKi = 80090.0f;
float pidKd = 0.0f;
float pidIntegrator = 0.0f;
float adcVoltage = 0.0f;
float adcRaw = 0.0f;
float pwmFrequencyHz = 0.0f;
float adcSampleRateHz = 0.0f;
int directionCode = 0;
int espTimeMs = 0;
boolean adcCalibrated = false;

ArrayList<Sample> samples = new ArrayList<Sample>();
ArrayList<Sample> graphSamples = new ArrayList<Sample>();
final int RECORD_LIMIT=60000;
boolean recordFull=false, advanced=false;
int acquisitionSegment=0;
long lastClockUs=-1, segmentStartUs=0;
Button settingsButton;
String lastReply="";
volatile String exportStatus="";
volatile boolean exporting=false;
ArrayList<Sample> visibleSamples = new ArrayList<Sample>();
float chartX=334, chartW;


Button connectButton;
Button refreshButton;
Button modeOpenButton;
Button modePidButton;
Button modeSpeedButton;
boolean speedPage = false, positionPage=false;
Button modePositionButton, zeroPositionButton;
Slider positionRefSlider, positionKpSlider, positionKiSlider, positionMaxSlider, positionAccelSlider, positionToleranceSlider;
PositionInfo position=new PositionInfo();
int lastPositionMillis=-1000;
int viewLoop(){return positionPage ? 2 : speedPage ? 1 : 0;}

SpeedInfo speed = new SpeedInfo();
int lastSpeedMillis = -1000;
Slider speedRefSlider, speedKpSlider, speedKiSlider, speedLimitSlider;

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
  surface.setTitle("ControlCurrentN20 - Corriente y velocidad");
  textFont(createFont("Arial", 15));

  initControls();
  refreshPorts();
}

void initControls() {
  connectButton = new Button(898, 44, 112, 34, "Conectar", color(90, 220, 160));
  refreshButton = new Button(1024, 44, 104, 34, "Refrescar", color(120, 160, 220));
  modeOpenButton = new Button(32, 154, 94, 36, "OPEN", color(90, 220, 160));
  modePidButton = new Button(138, 154, 94, 36, "PI I", color(80, 180, 255));
  modeSpeedButton = new Button(244,154,94,36,"PI vel",color(170,140,255));
  cwButton = new Button(32, 214, 94, 36, "CW", color(80, 180, 255));
  ccwButton = new Button(138, 214, 94, 36, "CCW", color(180, 140, 255));
  sendPidButton = new Button(32, 595, 122, 40, "Enviar PI", color(80, 180, 255));
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
  kpSlider = new Slider(32, 390, 350, 0, 1000, "Kp [%PWM/A]", 3);
  kiSlider = new Slider(32, 465, 350, 0, 150000, "Ki [%PWM/(A*s)]", 2);
  kdTimes100Slider = new Slider(32, 540, 350, 0, 1000, "Kd = 0 (PI, no editable)", 3);
  outputMaxSlider = new Slider(32, 725, 350, 0, 100, "Limite PWM max (%)", 1);

  refMilliAmpSlider.value = 100;
  kpSlider.value = pidKp;
  kiSlider.value = pidKi;
  kdTimes100Slider.value = pidKd * 100.0f;
  outputMaxSlider.value = 40;
  manualPwmSlider = new Slider(540, 350, 350, -100, 100, "PWM manual (%)", 2);
  manualPwmSlider.value = 0;
  applyPwmButton = new Button(1000, 336, 130, 34, "Aplicar PWM", color(90, 220, 160));
  speedRefSlider = new Slider(32,315,350,0,32,"Referencia velocidad (rad/s; magnitud)",3);
  speedKpSlider = new Slider(32,390,350,0,0.2f,"Kp velocidad [A eq. / (rad/s)]",5);
  speedKiSlider = new Slider(32,465,350,0,0.2f,"Ki velocidad [A eq. / rad]",5);
  speedLimitSlider = new Slider(32,540,350,1,1000,"Limite referencia CS (mA eq.)",0);
  speedKpSlider.value=0.0284f; speedKiSlider.value=0.01392f;
  speedLimitSlider.value=100;
  settingsButton=new Button(24,132,270,30,"Ajustes y diagnostico",color(120,150,190));
  refMilliAmpSlider.label="Referencia (mA eq. CS)";
  speedRefSlider.label="Referencia (rad/s)";
  outputMaxSlider.label="Techo PWM solicitado (%)";
  speedLimitSlider.label="Limite CS (mA equivalentes)";
  modePositionButton=new Button(208,86,86,34,"Posicion",color(180,150,255));
  zeroPositionButton=new Button(24,575,270,30,"Cero encoder (en STOP)",color(120,155,195));
  positionRefSlider=new Slider(24,218,164,-10000,10000,"Objetivo (rad; escribir en caja)",4);
  positionRefSlider.textOnly=true;
  positionKpSlider=new Slider(24,218,164,0,100,"Kp posicion [1/s]",4);positionKpSlider.value=6.166f;
  positionKiSlider=new Slider(24,286,164,0,10000,"Ki posicion [1/s^2]",3);positionKiSlider.value=3192;
  positionMaxSlider=new Slider(24,280,164,.01f,32,"Velocidad maxima (rad/s)",3);positionMaxSlider.value=2;
  positionAccelSlider=new Slider(24,410,164,.01f,100,"Rampa de velocidad (rad/s^2)",3);positionAccelSlider.value=5;
  positionToleranceSlider=new Slider(24,486,164,.0023f,1,"Tolerancia de posicion (rad)",4);positionToleranceSlider.value=.02f;
  layoutDashboard();
}

void draw() {
  layoutDashboard();
  visibleSamples=graphSnapshot();
  background(15,19,26);
  drawHeader(); drawControls(); drawLiveValues();
  drawDashboardPlots(); drawFooter();
}

void mousePressed() {
  activeTextSlider=null;
  if (stopButton.contains(mouseX,mouseY)) { sendCommand("STOP"); return; }
  if (connectButton.contains(mouseX,mouseY)) { connectSelectedPort(); return; }
  if (refreshButton.contains(mouseX,mouseY)) { refreshPorts(); return; }
  if (modePidButton.contains(mouseX,mouseY)) { speedPage=false;positionPage=false; return; }
  if (modeSpeedButton.contains(mouseX,mouseY)) { speedPage=true;positionPage=false; return; }
  if(modePositionButton.contains(mouseX,mouseY)){positionPage=true;speedPage=false;return;}
  if(positionPage && !advanced && zeroPositionButton.contains(mouseX,mouseY)){sendCommand("ENCODER,ZERO");return;}
  if (settingsButton.contains(mouseX,mouseY)) { advanced=!advanced; return; }
  if (exportButton.contains(mouseX,mouseY)) { exportData(); return; }
  if (clearButton.contains(mouseX,mouseY)) { clearSamples(); return; }
  if (!advanced) {
    if (!positionPage && cwButton.contains(mouseX,mouseY)) { selectDirection("CW"); return; }
    if (!positionPage && ccwButton.contains(mouseX,mouseY)) { selectDirection("CCW"); return; }
    if (sendRefButton.contains(mouseX,mouseY)) {
      if (positionPage) sendPositionReference(); else if (speedPage) sendSpeedReference(); else sendCurrentReference(); return;
    }
  } else {
    if (sendPidButton.contains(mouseX,mouseY)) {
      if (positionPage) sendPositionParameters(); else if (speedPage) sendSpeedParameters(); else sendPidParameters(); return;
    }
    if (!positionPage && applyPwmButton.contains(mouseX,mouseY)) {
      sendCommand("SET,"+Float.toString(manualPwmSlider.value)); return;
    }
  }
  for (Slider slider:visibleSliders()) slider.mousePressed();
}
Slider[] visibleSliders() {
  if(positionPage) return advanced ? new Slider[]{positionKpSlider,positionKiSlider,positionAccelSlider,positionToleranceSlider} :
    new Slider[]{positionRefSlider,positionMaxSlider,speedLimitSlider,outputMaxSlider};
  if (advanced) return new Slider[]{speedPage ? speedKpSlider : kpSlider,
    speedPage ? speedKiSlider : kiSlider,manualPwmSlider};
  return speedPage ? new Slider[]{speedRefSlider,outputMaxSlider,speedLimitSlider} :
    new Slider[]{refMilliAmpSlider,outputMaxSlider};
}
void mouseDragged() { for (Slider slider:visibleSliders()) slider.mouseDragged(); }
void mouseReleased() { for (Slider slider:visibleSliders()) slider.mouseReleased(); }

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
    if (positionPage) sendPositionParameters(); else if (speedPage) sendSpeedParameters(); else sendPidParameters();
  } else if (key == 'i' || key == 'I') {
    if (positionPage) sendPositionReference(); else if (speedPage) sendSpeedReference(); else sendCurrentReference();
  } else if (key == 'm' || key == 'M') {
    if (!controlModeText.equals("OPEN")) sendCommand("STOP");
    else if(positionPage) sendPositionReference(); else if (speedPage) activateSpeed(); else activatePi();
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
    if(line.startsWith("ACK,LIMIT,")) {
      try {
        String[] ack=split(line,',');float v=Float.parseFloat(ack[3]);
        pwmLimitConfirmed=ack.length==4 && Float.isFinite(v) && v>=0 && v<=100 ? v : Float.NaN;
      } catch(RuntimeException e){pwmLimitConfirmed=Float.NaN;}
    }
    status = line; lastReply=line;
    return;
  }

  if(line.startsWith("POSITION,")) {
    try{position=new PositionInfo(split(line,','));lastPositionMillis=millis();}
    catch(RuntimeException e){position=new PositionInfo();status="POSITION incompatible";}
    return;
  }
  if (line.startsWith("SPEED,")) {
    try { speed = new SpeedInfo(split(line,',')); lastSpeedMillis=millis(); }
    catch (RuntimeException e) { speed=new SpeedInfo(); status="Trama SPEED incompatible"; }
    return;
  }
  if (!line.startsWith("DATA,")) {
    return;
  }

  String[] data = split(line, ',');
  lastLine = line;
  long adcStamp;
  try {
    SensingInfo nextSensing = new SensingInfo(data);
    EncoderInfo nextEncoder = new EncoderInfo(data);
    adcStamp=Long.parseLong(data[24]);
    if(adcStamp<=0) throw new IllegalArgumentException("Tiempo ADC invalido");
    espTimeMs = (int)Long.parseLong(data[1]);
    adcRaw = sensingFloat(data[2]);
    adcVoltage = sensingFloat(data[3]);
    currentRawA = sensingFloat(data[4]);
    directionCode = Integer.parseInt(data[5]);
    pwmFrequencyHz = sensingFloat(data[10]);
    adcSampleRateHz = sensingFloat(data[11]);
    controlModeText = data[14];
    currentRefA = sensingFloat(data[15]);
    currentErrorA = sensingFloat(data[16]);
    controlOutputPercent = sensingFloat(data[17]);
    dutyPercent = sensingFloat(data[18]);
    pidKp = sensingFloat(data[19]);
    pidKi = sensingFloat(data[20]);
    pidKd = sensingFloat(data[21]);
    pidIntegrator = sensingFloat(data[22]);
    adcCalibrated = Integer.parseInt(data[23]) == 1;
    if (!Float.isFinite(dutyPercent) || !Float.isFinite(controlOutputPercent))
      throw new IllegalArgumentException("Telemetria invalida");
    sensing = nextSensing;
    encoder = nextEncoder;
  } catch (RuntimeException e) {
    invalidateTelemetry();
    return;
  }
  lastDataMillis = millis();
  status = "DATA v6 | " + (speedFresh() ? "SPEED v1 disponible" : "sin telemetria de velocidad PI");
  currentMeanA = sensing.primaryCurrent(currentRawA);
  if (!sensing.valid) currentErrorA = Float.NaN;
  Sample next=new Sample(millis(), espTimeMs, currentRawA, currentRefA,
                       currentErrorA, controlOutputPercent, dutyPercent, sensing, encoder);
  next.adcTimeUs=adcStamp;
  addSample(next);
}

/** Limpia los valores actuales cuando el protocolo no coincide, sin inventar registros. */
void invalidateTelemetry() {
  status = "Trama incompatible: se requiere DATA v6 completo";
  sensing = new SensingInfo();
  encoder = new EncoderInfo();
  adcRaw = adcVoltage = currentRawA = currentMeanA = currentRefA = currentErrorA = Float.NaN;
  dutyPercent = controlOutputPercent = pidKp = pidKi = pidKd = pidIntegrator = Float.NaN;
  pwmFrequencyHz = adcSampleRateHz = Float.NaN;
  adcCalibrated = false;
  directionCode = 0;
  controlModeText = "sin lectura";
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

  synchronized(samples) { acquisitionSegment++; lastClockUs=-1; }
  pwmLimitConfirmed=Float.NaN;
  esp32 = new Serial(this, ports[selectedPort], 460800);
  encoder = new EncoderInfo();
  speed = new SpeedInfo(); lastSpeedMillis=-1000;
  position=new PositionInfo();lastPositionMillis=-1000;
  esp32.clear();
  esp32.bufferUntil('\n');
  serialConnected = true;
  lastDataMillis = millis();
  status = "Conectado a " + ports[selectedPort];
}

void sendPidParameters() {
  float kp = kpSlider.value;
  float ki = kiSlider.value;
  float kd = 0.0f; // PI: el protocolo conserva el campo Kd, siempre cero.
  kdTimes100Slider.value = 0;
  float limitMax = outputMaxSlider.value;
  sendCommand("PID," + wireNumber(kp, 0, 4) + "," + wireNumber(ki, 0, 4) + "," + wireNumber(kd, 0, 4));
  sendCommand("LIMIT,0," + wireNumber(limitMax, 0, 3));
}

void sendCurrentReference() {
  if(controlModeText.equals("POSITION")){status="STOP antes de pasar de posicion a corriente";lastReply="";return;}
  float iref = refMilliAmpSlider.value / 1000.0f;
  sendCommand("LIMIT,0," + wireNumber(outputMaxSlider.value, 0, 3));
  if (controlModeText.equals("SPEED")) activatePi();
  sendCommand("IREF," + wireNumber(iref, 0, 4));
  if (!controlModeText.equals("PID") && !controlModeText.equals("SPEED")) activatePi();
}

void activatePi() {
  if(controlModeText.equals("POSITION")){status="STOP antes de cambiar de modo";lastReply="";return;}
  // Apply the displayed ceiling before enabling control (initially 40%).
  sendCommand("LIMIT,0," + wireNumber(outputMaxSlider.value, 0, 3));
  sendCommand("MODE,PID," + direction);
}

boolean speedFresh() { return speed.supported && millis()-lastSpeedMillis<500; }

void selectDirection(String next) {
  if (!controlModeText.equals("OPEN") && !direction.equals(next)) {
    status="STOP y esperar a detener el eje antes de cambiar direccion"; return;
  }
  direction=next; sendCommand("DIR,"+direction);
}

void sendSpeedParameters() {
  sendCommand("SPI,"+wireNumber(speedKpSlider.value,0,5)+","+wireNumber(speedKiSlider.value,0,5));
  sendCommand("ILIMIT,"+wireNumber(speedLimitSlider.value/1000.0f,0,4));
  sendCommand("LIMIT,0,"+wireNumber(outputMaxSlider.value,0,3));
}

void activateSpeed() {
  if(controlModeText.equals("POSITION")){status="STOP antes de pasar de posicion a velocidad";lastReply="";return;}
  if (!speedFresh()) { status="Velocidad requiere firmware con SPEED v1 y datos recientes"; return; }
  sendCommand("LIMIT,0,"+wireNumber(outputMaxSlider.value,0,3));
  sendCommand("ILIMIT,"+wireNumber(speedLimitSlider.value/1000.0f,0,4));
  sendCommand("WREF,"+wireNumber(speedRefSlider.value,0,4));
  sendCommand("MODE,SPEED,"+direction);
}

void sendSpeedReference() {
  if (!speedFresh()) { status="Sin SPEED v1 reciente; no se activa velocidad"; return; }
  if (!controlModeText.equals("SPEED")) { activateSpeed(); return; }
  sendCommand("LIMIT,0,"+wireNumber(outputMaxSlider.value,0,3));
  sendCommand("ILIMIT,"+wireNumber(speedLimitSlider.value/1000.0f,0,4));
  sendCommand("WREF,"+wireNumber(speedRefSlider.value,0,4));
}

boolean positionFresh(){return position.supported && millis()-lastPositionMillis<500;}
void sendPositionParameters(){
  if(controlModeText.equals("POSITION")){positionNotice("STOP antes de cambiar ganancias o limites de posicion");return;}
  sendCommand("PPI,"+wireNumber(positionKpSlider.value,0,4)+","+wireNumber(positionKiSlider.value,0,4));
  sendPositionLimits();
}
void sendPositionLimits(){
  sendCommand("PLIMIT,"+wireNumber(positionMaxSlider.value,0,4)+","+
    wireNumber(positionAccelSlider.value,0,4)+","+wireNumber(positionToleranceSlider.value,0,4));
}
void sendPositionReference(){
  lastReply="";
  if(!positionFresh() || !speedFresh()){positionNotice("Posicion requiere firmware POSITION v1 reciente");return;}
  if(!controlModeText.equals("POSITION") && !controlModeText.equals("OPEN")){positionNotice("STOP antes de activar posicion");return;}
  if(controlModeText.equals("POSITION")) {
    if(abs(position.maxSpeed-positionMaxSlider.value)>.0001f || abs(position.accel-positionAccelSlider.value)>.0001f ||
       abs(position.tolerance-positionToleranceSlider.value)>.0001f){positionNotice("STOP para aplicar nuevos limites de posicion");return;}
  } else sendPositionLimits();
  sendCommand("LIMIT,0,"+wireNumber(outputMaxSlider.value,0,3));
  sendCommand("ILIMIT,"+wireNumber(speedLimitSlider.value/1000,0,4));
  sendCommand("PREF,"+wireNumber(positionRefSlider.value,0,5));
  if(!controlModeText.equals("POSITION"))sendCommand("MODE,POSITION");
}
void positionNotice(String message){status=message;lastReply="UI, "+message;}

// Serial always uses a decimal point, regardless of Windows/Processing locale.
String wireNumber(float value, int ignored, int decimals) {
  return String.format(java.util.Locale.US,"%."+decimals+"f",value);
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
    if (lastClockUs>=0 && sample.adcTimeUs<lastClockUs) {
      acquisitionSegment++; lastClockUs=-1;pwmLimitConfirmed=Float.NaN;sample.pwmLimit=Float.NaN;
    }
    if (lastClockUs<0) segmentStartUs=sample.adcTimeUs;
    sample.segment=acquisitionSegment;
    sample.elapsedS=(sample.adcTimeUs-segmentStartUs)/1e6;
    lastClockUs=sample.adcTimeUs;
    graphSamples.add(sample);
    if (graphSamples.size()>2200) graphSamples.remove(0);
    if (samples.size()<RECORD_LIMIT) samples.add(sample);
    else recordFull=true;
  }
}
void clearSamples() {
  synchronized(samples) {
    samples.clear(); graphSamples.clear(); recordFull=false;
    acquisitionSegment++; lastClockUs=-1;
  }
  status="Nuevo registro; exporta antes de limpiar si deseas conservar el anterior";
  exportStatus="";
}
ArrayList<Sample> sampleSnapshot() {
  synchronized(samples) { return new ArrayList<Sample>(samples); }
}
ArrayList<Sample> graphSnapshot() {
  synchronized(samples) { return new ArrayList<Sample>(graphSamples); }
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
    float primary=sample.currentMeanA;
    if (Float.isFinite(primary)) maxA=max(maxA,primary);
    if (Float.isFinite(sample.currentRefA)) maxA=max(maxA,sample.currentRefA);
  }
  return ceil(maxA * 10.0f + 1.0f) / 10.0f;
}

void exportData() {
  if(exporting)return;
  ArrayList<Sample> exportSamples = sampleSnapshot();
  if (exportSamples.size() == 0) {
    status = "No hay muestras para exportar";
    return;
  }

  String filename = "control_n20_v3_"+java.time.LocalDateTime.now().format(
    java.time.format.DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS"))+".csv";
  File output = new File(sketchPath("exports"), filename);
  output.getParentFile().mkdirs();
  exporting=true;exportStatus="Exportando "+exportSamples.size()+" muestras...";
  new Thread(() -> {
    try {
      writeCsvSnapshot(exportSamples,output);
      exportStatus="Exportado: "+output.getName()+" | "+exportSamples.size()+" filas";
    } catch(Exception e) { exportStatus="Error CSV: "+e.getMessage(); }
    finally { exporting=false; }
  },"csv-export").start();
}

String csvCell(String value) { return "\""+(value==null ? "" : value.replace("\"","\"\""))+"\""; }
/** Bounded CSV batches keep ten-minute exports from building a huge Table in RAM. */
void writeCsvSnapshot(ArrayList<Sample> snapshot,File output) throws java.io.IOException {
  try(java.io.BufferedWriter writer=java.nio.file.Files.newBufferedWriter(output.toPath(),java.nio.charset.StandardCharsets.UTF_8)) {
    for(int start=0;start<snapshot.size();start+=256) {
      Table part=sampleTable(snapshot,start,min(snapshot.size(),start+256));
      if(start==0) {for(int c=0;c<part.getColumnCount();c++){if(c>0)writer.write(',');writer.write(csvCell(part.getColumnTitle(c)));}writer.newLine();}
      for(TableRow row:part.rows()) {
        for(int c=0;c<part.getColumnCount();c++){if(c>0)writer.write(',');writer.write(csvCell(row.getString(c)));}
        writer.newLine();
      }
    }
  }
}
Table sampleTable(ArrayList<Sample> exportSamples) {
  return sampleTable(exportSamples,0,exportSamples.size());
}
Table sampleTable(ArrayList<Sample> exportSamples,int from,int until) {
  Table table = new Table();
  for (String c:new String[]{"csv_schema_version","record_index","segment_id","segment_time_s","adc_time_us",
    "loop_unit","loop_reference","loop_measured","loop_error","loop_valid","metrics_valid_s","rmse_5s",
    "mae_5s","abs_error_max_5s","pwm_at_confirmed_limit_pct_5s"}) table.addColumn(c);
  table.addColumn("pc_time_ms");
  table.addColumn("esp_time_ms");
  table.addColumn("current_raw_a");
  table.addColumn("current_ref_a");
  table.addColumn("current_error_a");
  table.addColumn("control_output_percent");
  table.addColumn("duty_percent");
  table.addColumn("kp");
  table.addColumn("ki");
  table.addColumn("kd");
  table.addColumn("pwm_limit_max_percent");
  table.addColumn("control_mode");
  new SpeedInfo().addColumns(table);
  new PositionInfo().addColumns(table);
  new SensingInfo().addColumns(table);
  new EncoderInfo().addColumns(table);

  for (int recordIndex=from;recordIndex<until;recordIndex++) {
    Sample sample=exportSamples.get(recordIndex);
    TableRow row = table.addRow();
    int velocity=sample.mode.equals("POSITION") ? 2 : sample.mode.equals("SPEED") ? 1 : 0;
    Metrics m=metrics(exportSamples,recordIndex,velocity);
    row.setInt("csv_schema_version",3); row.setInt("record_index",recordIndex);
    row.setInt("segment_id",sample.segment); row.setDouble("segment_time_s",sample.elapsedS);
    row.setString("adc_time_us",Long.toString(sample.adcTimeUs));
    row.setString("loop_unit",velocity==2 ? "rad" : velocity==1 ? "rad/s" : sample.mode.equals("PID") ? "A_eq_CS" : "");
    row.setString("loop_reference",finiteCell(loopReference(sample,velocity)));
    row.setString("loop_measured",finiteCell(loopMeasurement(sample,velocity)));
    row.setString("loop_error",finiteCell(loopError(sample,velocity)));
    row.setInt("loop_valid",Float.isFinite(loopError(sample,velocity)) ? 1 : 0);
    row.setDouble("metrics_valid_s",m.seconds);
    row.setString("rmse_5s",finiteCell(m.rmse())); row.setString("mae_5s",finiteCell(m.mae()));
    row.setString("abs_error_max_5s",finiteCell(m.peak()));
    row.setString("pwm_at_confirmed_limit_pct_5s",finiteCell(m.saturation()));
    row.setInt("pc_time_ms", sample.pcTimeMs);
    row.setInt("esp_time_ms", sample.espTimeMs);
    row.setString("current_raw_a", finiteCell(sample.currentRawA));
    row.setFloat("current_ref_a", sample.currentRefA);
    row.setString("current_error_a", finiteCell(sample.currentErrorA));
    row.setFloat("control_output_percent", sample.controlOutputPercent);
    row.setFloat("duty_percent", sample.dutyPercent);
    row.setFloat("kp", sample.kp);
    row.setFloat("ki", sample.ki);
    row.setFloat("kd", sample.kd);
    row.setFloat("pwm_limit_max_percent", sample.pwmLimit);
    row.setString("control_mode",sample.mode);
    sample.speedInfo.writeRow(row);
    sample.positionInfo.writeRow(row);
    sample.sensing.writeRow(row);
    sample.encoder.writeRow(row);
  }

  return table;
}

void place(Button b,float x,float y,float w,float h) { b.x=x;b.y=y;b.w=w;b.h=h; }
void placeSlider(Slider s,float y) { s.x=24;s.y=y;s.w=164;s.inputW=90; }
void layoutDashboard() {
  chartX=334; chartW=max(380,width-chartX-24);
  place(connectButton,width-338,20,106,34); place(refreshButton,width-220,20,94,34);
  place(stopButton,width-114,20,90,34);
  place(modePidButton,24,86,86,34); place(modeSpeedButton,116,86,86,34);place(modePositionButton,208,86,86,34);
  modePidButton.label="Corriente"; modeSpeedButton.label="Velocidad";
  place(settingsButton,24,134,270,30);
  place(cwButton,24,256,130,32); place(ccwButton,164,256,130,32);
  placeSlider(refMilliAmpSlider,218);placeSlider(speedRefSlider,218);
  placeSlider(outputMaxSlider,332);placeSlider(speedLimitSlider,408);
  place(sendRefButton,24,452,270,40);sendRefButton.label="Aplicar referencia / activar";
  placeSlider(kpSlider,218);placeSlider(speedKpSlider,218);
  placeSlider(kiSlider,286);placeSlider(speedKiSlider,286);
  place(sendPidButton,24,330,270,34);sendPidButton.label="Aplicar ganancias";
  placeSlider(manualPwmSlider,420);place(applyPwmButton,24,454,270,34);
  placeSlider(positionRefSlider,218);placeSlider(positionMaxSlider,286);
  placeSlider(positionKpSlider,218);placeSlider(positionKiSlider,286);
  placeSlider(positionAccelSlider,410);placeSlider(positionToleranceSlider,486);
  if(positionPage && !advanced){placeSlider(speedLimitSlider,354);placeSlider(outputMaxSlider,422);place(sendRefButton,24,464,270,36);}
  place(exportButton,24,height-114,270,36); exportButton.label="Exportar ensayo CSV";
  place(clearButton,24,height-66,270,30);clearButton.label="Nuevo registro (borra actual)";
}
void drawHeader() {
  fill(238);textSize(23);text("N20 / Control en cascada",24,37);
  fill(145,158,177);textSize(12);text("ESP32  ·  CS media DMA  ·  rad/s",24,58);
  fill(serialConnected && millis()-lastDataMillis<500 ? color(90,220,160) : color(240,183,83));
  textSize(13);
  String port=ports.length>0 ? ports[selectedPort] : "Sin puerto";
  text(port+"  /  "+(serialConnected && millis()-lastDataMillis<500 ? controlModeText : "sin datos recientes"),width-550,41);
  connectButton.draw();refreshButton.draw();stopButton.draw();
}
void drawControls() {
  noStroke();fill(23,29,39);rect(12,74,294,height-100,10);
  modePidButton.active=!speedPage && !positionPage;modeSpeedButton.active=speedPage;modePositionButton.active=positionPage;
  modePidButton.draw();modeSpeedButton.draw();modePositionButton.draw();
  settingsButton.label=advanced ? "Volver a consignas" : "Ajustes y diagnostico";
  settingsButton.draw();
  for (Slider sl:visibleSliders()) sl.draw();
  if (!advanced) {
    cwButton.active=direction.equals("CW");ccwButton.active=direction.equals("CCW");
    if(!positionPage){cwButton.draw();ccwButton.draw();}sendRefButton.draw();
    if(positionPage)zeroPositionButton.draw();
    fill(151,167,188);textSize(12);
    text("Activo en ESP32: "+controlModeText,24,523);
    text(positionPage ? "Pos -> vel -> corriente -> PWM" : speedPage ? "Velocidad -> corriente -> PWM" : "Corriente -> PWM",24,545);
    if(positionPage){
      text(positionFresh() ? (controlModeText.equals("POSITION") ? position.stateText() : "Listo; activacion explicita") : "Esperando POSITION v1",24,563);
      text("Objetivo respecto al cero; sin homing.",24,627);
    } else {text("Cambiar vista no activa el motor.",24,571);text("Espacio: STOP / coast",24,591);}
  } else {
    sendPidButton.draw();if(!positionPage)applyPwmButton.draw();fill(151,167,188);textSize(12);
    text(positionPage ? "Cambiar parametros requiere STOP." : "PWM manual activa modo OPEN.",24,positionPage ? 524 : 511);
    text(positionPage ? (positionFresh() ? "PI pos: "+nf(position.kp,0,3)+" / "+nf(position.ki,0,1) : "PI pos sin datos") : "PI I recibido: "+nf(pidKp,0,3)+" / "+nf(pidKi,0,0),24,546);
    text("PI vel: "+(speedFresh() ? nf(speed.kp,0,5)+" / "+nf(speed.ki,0,5) : "sin datos"),24,559);
    text("ADC: "+rateText(sensing.adcHz)+" Hz | bloque "+sensing.dmaBlockSamples,24,580);
    text("Consulta: "+rateText(sensing.acquisitionHz)+" Hz",24,599);
  }
  exportButton.draw();clearButton.draw();
}
String metricText(float v) { return Float.isFinite(v) ? nf(v,0,3) : "--"; }
void drawLiveValues() {
  ArrayList<Sample> source=graphSnapshot();
  Metrics m=metrics(source,source.size()-1,viewLoop());
  String unit=positionPage ? "rad" : speedPage ? "rad/s" : "mA eq.";
  float factor=positionPage || speedPage ? 1 : 1000;
  String[] titles={"ERROR RMS", "ERROR ABS. MEDIO", "ERROR MAX. ABS.", "TIEMPO EN TECHO*"};
  String[] vals={metricText(m.rmse()*factor)+" "+unit,metricText(m.mae()*factor)+" "+unit,
    metricText(m.peak()*factor)+" "+unit,metricText(m.saturation())+" %"};
  float w=(max(380,width-334-24)-30)/4;
  for (int j=0;j<4;j++) {
    float x=334+j*(w+10);noStroke();fill(25,33,44);rect(x,86,w,75,8);
    fill(157,173,194);textSize(11);text(titles[j],x+12,108);
    fill(231,238,247);textSize(w<200 ? 16 : 21);text(vals[j],x+12,140);
  }
  fill(151,167,188);textSize(11);
  text("Ultimos 5 s del modo activo | cobertura valida: "+nf((float)m.seconds,0,2)+" s | *PWM frente al techo confirmado por ESP32",334,179);
}
void drawDashboardPlots() {
  float top=198, available=height-top-57, gap=14;
  float first=(available-2*gap)*.49f, second=(available-2*gap)*.23f, third=(available-2*gap)*.28f;
  dashboardPlot(chartX,top,chartW,first,0,positionPage ? "Posicion de salida" : speedPage ? "Velocidad de salida" : "Corriente CS equivalente");
  dashboardPlot(chartX,top+first+gap,chartW,second,1,"Error de seguimiento");
  float y=top+first+second+2*gap;
  if(positionPage){
    float w=(chartW-2*gap)/3;
    dashboardPlot(chartX,y,w,third,4,"Velocidad");
    dashboardPlot(chartX+w+gap,y,w,third,3,"CS interior");
    dashboardPlot(chartX+2*(w+gap),y,w,third,2,"PWM");
  } else if (speedPage) {
    dashboardPlot(chartX,y,(chartW-gap)/2,third,2,"PWM aplicado");
    dashboardPlot(chartX+(chartW+gap)/2,y,(chartW-gap)/2,third,3,"Lazo interior / CS");
  } else dashboardPlot(chartX,y,chartW,third,2,"PWM aplicado");
}
float plotValue(Sample s,int signal) {
  if(signal==0) return loopMeasurement(s,viewLoop());
  if(signal==1) return loopReference(s,viewLoop());
  if(signal==2) return loopError(s,viewLoop());
  if(signal==3) return s.dutyPercent;
  if(signal==4) return s.currentMeanA;
  if(signal==6)return loopMeasurement(s,1);
  if(signal==7)return loopReference(s,1);
  return s.currentRefA;
}
double plotTime(Sample s) { return s.adcTimeUs/1e6; }
void dashboardPlot(float x,float y,float w,float h,int kind,String title) {
  noStroke();fill(22,28,38);rect(x,y,w,h,10);fill(226,234,245);textSize(15);text(title,x+16,y+25);
  boolean pair=kind==0 || kind==3 || kind==4;
  int sig=kind==0 ? 0 : kind==1 ? 2 : kind==2 ? 3 : kind==4 ? 6 : 4;
  String unit=kind==2 ? "%" : kind==4 ? "rad/s" : kind==3 ? "A eq." : positionPage ? "rad" : speedPage ? "rad/s" : "A eq.";
  float lo=0,hi=kind==2 ? 100 : kind==4 ? 1 : kind==3 || (!speedPage && !positionPage) ? .01f : 1;
  int segment=visibleSamples.isEmpty() ? -1 : visibleSamples.get(visibleSamples.size()-1).segment;
  double end=visibleSamples.isEmpty() ? 20 : plotTime(visibleSamples.get(visibleSamples.size()-1));
  double start=Math.max(0,end-20);
  for(Sample s:visibleSamples) if(s.segment==segment && plotTime(s)>=start) {
    for(int k=0;k<(pair ? 2 : 1);k++) {float v=plotValue(s,sig+k);if(Float.isFinite(v)){lo=min(lo,v);hi=max(hi,v);}}
  }
  if(kind==2) lo=-100;
  else {float pad=max((hi-lo)*.12f,.001f);hi+=pad; if(lo<0)lo-=pad;}
  if(kind==1) {hi=max(abs(lo),abs(hi));lo=-hi;}
  float px=x+65,py=y+46,pw=w-84,ph=max(25,h-77);
  textSize(10);
  int ticks=ph<80 ? 2 : 4;
  for(int n=0;n<=ticks;n++) {
    float gy=py+n*ph/ticks;stroke(43,53,68);line(px,gy,px+pw,gy);
    fill(139,156,179);textAlign(RIGHT,CENTER);text(nf(lerp(hi,lo,n/(float)ticks),0,kind==2 ? 0 : 3),px-8,gy);
  }
  textAlign(LEFT,BASELINE);fill(139,156,179);text(unit,x+12,y+41);
  for(int n=0;n<=4;n++) {float gx=px+n*pw/4;fill(139,156,179);textAlign(CENTER,TOP);text(nf((float)(-min(20,(float)end)*(1-n/4.0)),0,1)+" s",gx,py+ph+6);}
  textAlign(LEFT,BASELINE);
  for(int k=0;k<(pair ? 2 : 1);k++) {
    stroke(k==0 ? color(75,202,220) : color(255,192,102));strokeWeight(k==0 ? 1.7f : 1.2f);noFill();
    boolean open=false;double previous=-1;
    for(Sample s:visibleSamples) {
      double t=plotTime(s);if(s.segment!=segment || t<start)continue;
      float v=plotValue(s,sig+k);
      if(!Float.isFinite(v) || (previous>=0 && (t<=previous || t-previous>.1))) {if(open)endShape();open=false;}
      previous=t;if(!Float.isFinite(v))continue;
      if(!open){beginShape();open=true;}
      vertex(px+(float)((t-start)/max(.001f,(float)(end-start)))*pw,py+ph-(v-lo)/(hi-lo)*ph);
    }
    if(open)endShape();strokeWeight(1);
  }
  fill(75,202,220);textSize(11);text(pair ? "Medida" : kind==1 ? "Referencia - medida" : "PWM",x+w-180,y+24);
  if(pair){fill(255,192,102);text("Referencia",x+w-95,y+24);}
}

void drawYAxis(float x, float y, float w, float h, float axisMax, String unit) {
  stroke(70);
  fill(170);
  textSize(12);
  textAlign(RIGHT, CENTER);

  for (int j = 0; j <= 4; j++) {
    float value = map(j, 0, 4, axisMax, (unit.equals("%") || unit.equals("rad/s")) ? -axisMax : 0);
    float gy = map(value, (unit.equals("%") || unit.equals("rad/s")) ? -axisMax : 0, axisMax, y + h, y);
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
    float value = sample.currentMeanA;
    if (signal == 1) {
      value = sample.currentRefA;
    } else if (signal == 2) {
      value = sample.controlOutputPercent;
    } else if (signal == 3) {
      value = sample.dutyPercent;
    } else if (signal == 4) {
      value = sample.encoder.speedValid ? (float)sample.encoder.speedRadS : Float.NaN;
    } else if (signal == 5) {
      value = sample.speedInfo.supported && sample.mode.equals("SPEED") ? sample.speedInfo.reference*sample.speedInfo.direction : Float.NaN;
    }

    if (!Float.isFinite(value)) { if (segment) endShape(); segment=false; continue; }
    if (!segment) { beginShape(); segment=true; }
    float px = map(sample.pcTimeMs - t0, 0, span, x, x + w);
    float py = map(value, signal >= 2 ? -axisMax : 0, axisMax, y + h, y);
    vertex(constrain(px, x, x + w), constrain(py, y, y + h));
  }
  if (segment) endShape();
  strokeWeight(1);
}

void drawFooter() {
  fill(recordFull ? color(255,180,90) : color(153,168,188));textSize(11);
  int count; synchronized(samples){count=samples.size();}
  String summary=recordFull ? "REGISTRO LLENO (60000): exporta y crea otro. La vista sigue en vivo." :
    "Registro: "+count+" / 60000 muestras | Vista: ultimos 20 s | CSV: ensayo completo";
  text(summary,334,height-30);
  String message=lastReply.startsWith("ERR,") || lastReply.startsWith("UI,") ? lastReply : exportStatus.length()>0 ? exportStatus : lastReply.length()>0 ? lastReply : status;
  text(message.length()>130 ? message.substring(0,130)+"..." : message,334,height-12);
}

// Metrics use device time, contiguous valid intervals <=100 ms, and one mode/segment.
float loopReference(Sample s,boolean v){return loopReference(s,v ? 1 : 0);}
float loopMeasurement(Sample s,boolean v){return loopMeasurement(s,v ? 1 : 0);}
float loopError(Sample s,boolean v){return loopError(s,v ? 1 : 0);}
float loopReference(Sample s,int loop) {
  if(loop==2)return s.mode.equals("POSITION") && s.positionInfo.supported && s.positionInfo.referenceId==s.encoder.referenceId ? (float)s.positionInfo.reference : Float.NaN;
  boolean closed=s.mode.equals("PID") || s.mode.equals("SPEED") || s.mode.equals("POSITION");
  if(!closed || (loop==1 && s.mode.equals("PID")))return Float.NaN;
  return loop==1 ? (s.speedInfo.supported ? s.speedInfo.reference*s.speedInfo.direction : Float.NaN) : s.currentRefA;
}
float loopMeasurement(Sample s,int loop) {
  if(loop==2)return s.mode.equals("POSITION") && s.encoder.supported ? (float)s.encoder.positionRad : Float.NaN;
  boolean closed=s.mode.equals("PID") || s.mode.equals("SPEED") || s.mode.equals("POSITION");
  if(!closed || (loop==1 && s.mode.equals("PID")))return Float.NaN;
  return loop==1 ? (s.encoder.speedValid ? (float)s.encoder.speedRadS : Float.NaN) : s.currentMeanA;
}
float loopError(Sample s,int loop){
  if(loop==2 && Float.isFinite(loopReference(s,loop)) && s.encoder.supported)
    return (float)(s.positionInfo.reference-s.encoder.positionRad);
  return loopReference(s,loop)-loopMeasurement(s,loop);
}
double metricTime(Sample s,int loop){return (loop>0 ? s.encoder.timeUs : s.adcTimeUs)/1e6;}
Metrics metrics(ArrayList<Sample> data,int endIndex,boolean v){return metrics(data,endIndex,v ? 1 : 0);}
class Metrics {
  double seconds,sumAbs,sumSq,maximum,satSeconds,satValidSeconds;
  float rmse(){return seconds>0 ? (float)Math.sqrt(sumSq/seconds) : Float.NaN;}
  float mae(){return seconds>0 ? (float)(sumAbs/seconds) : Float.NaN;}
  float peak(){return seconds>0 ? (float)maximum : Float.NaN;}
  float saturation(){return satValidSeconds>0 ? (float)(100*satSeconds/satValidSeconds) : Float.NaN;}
}
Metrics metrics(ArrayList<Sample> data,int endIndex,int velocity) {
  Metrics m=new Metrics();if(endIndex<0)return m;
  Sample end=data.get(endIndex);if(!Float.isFinite(loopReference(end,velocity)))return m;
  double edge=metricTime(end,velocity)-5;
  for(int n=endIndex;n>0;n--) {
    Sample b=data.get(n),a=data.get(n-1);
    if(a.segment!=end.segment || !a.mode.equals(end.mode) || !b.mode.equals(end.mode) || (velocity>0 && a.encoder.referenceId!=end.encoder.referenceId))break;
    double ta=metricTime(a,velocity),tb=metricTime(b,velocity),dt=tb-ta;
    if(tb<=edge)break;
    float ea=loopError(a,velocity),eb=loopError(b,velocity);
    if(dt<=0 || dt>.1 || !Float.isFinite(ea) || !Float.isFinite(eb))continue;
    double portion=tb-Math.max(ta,edge);
    // Piecewise-constant received error; do not interpolate across reference steps.
    m.seconds+=portion;m.sumAbs+=Math.abs(eb)*portion;m.sumSq+=eb*eb*portion;
    m.maximum=Math.max(m.maximum,Math.abs(eb));
    if(Float.isFinite(b.pwmLimit) && b.pwmLimit>0 && Float.isFinite(b.dutyPercent)) {
      m.satValidSeconds+=portion;if(Math.abs(b.dutyPercent)>=b.pwmLimit-.1)m.satSeconds+=portion;
    }
  }
  return m;
}

class Sample {
  long adcTimeUs;
  int segment;
  double elapsedS;
  int pcTimeMs;
  int espTimeMs;
  float currentRawA;
  float currentMeanA;
  float currentRefA;
  float currentErrorA;
  float controlOutputPercent;
  float dutyPercent;
  SensingInfo sensing;
  EncoderInfo encoder;
  SpeedInfo speedInfo;
  PositionInfo positionInfo;
  float kp,ki,kd,pwmLimit;
  String mode;

  Sample(int pcTimeMs, int espTimeMs, float currentRawA,
         float currentRefA, float currentErrorA, float controlOutputPercent,
         float dutyPercent, SensingInfo sensing, EncoderInfo encoder) {
    this.pcTimeMs = pcTimeMs;
    this.espTimeMs = espTimeMs;
    this.adcTimeUs=espTimeMs*1000L;
    this.currentRawA = currentRawA;
    this.currentMeanA = sensing.primaryCurrent(currentRawA);
    this.currentRefA = currentRefA;
    this.currentErrorA = currentErrorA;
    this.controlOutputPercent = controlOutputPercent;
    this.dutyPercent = dutyPercent;
    this.sensing = sensing;
    this.encoder = encoder;
    this.speedInfo=speedFresh() ? speed : new SpeedInfo();
    this.positionInfo=positionFresh() ? position : new PositionInfo();
    this.kp=pidKp; this.ki=pidKi; this.kd=pidKd; this.mode=controlModeText;
    this.pwmLimit=pwmLimitConfirmed;
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
  boolean textOnly=false;
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
    if (!textOnly && mouseX >= x && mouseX <= x + w && mouseY >= y - 18 && mouseY <= y + 28) {
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
    if(!textOnly)ellipse(knobX, y + 6, 28, 28);

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

// DATA v6 has exactly 59 fields; other layouts must update firmware.
float sensingFloat(String value) {
  return value.equalsIgnoreCase("nan") ? Float.NaN : Float.parseFloat(value);
}

String finiteCell(float value) { return Float.isFinite(value) ? Float.toString(value) : ""; }
String currentText(float value) { return Float.isFinite(value) ? nf(value,1,4)+" A" : "sin lectura"; }
String rateText(float value) { return Float.isFinite(value) && value>=0 ? nf(value,0,0) : "--"; }

/** Valores SI calculados por ESP32. Este decodificador no calibra ni deriva conteos.
 * DATA[49..58] es la extension encoder v1; no se inventan angulos.
 */
class EncoderInfo {
  boolean present, supported, countPresent, speedValid;
  int version;
  long count, timeUs, relativeCount, speedWindowUs, countsPerOutputRev, referenceCount, referenceId;
  double positionRad=Double.NaN, speedRadS=Double.NaN;
  EncoderInfo() { }
  EncoderInfo(String[] fields) {
    if (fields.length!=59) throw new IllegalArgumentException("Encoder incompleto");
    count=Long.parseLong(fields[25]); countPresent=true; present=true;
    version=Integer.parseInt(fields[49]);
    if (version!=1) return;
    timeUs=Long.parseLong(fields[50]); relativeCount=Long.parseLong(fields[51]);
    positionRad=Double.parseDouble(fields[52]);
    speedRadS=fields[53].equalsIgnoreCase("nan") ? Double.NaN : Double.parseDouble(fields[53]);
    int valid=Integer.parseInt(fields[54]);
    speedWindowUs=Long.parseLong(fields[55]); countsPerOutputRev=Long.parseLong(fields[56]);
    referenceCount=Long.parseLong(fields[57]); referenceId=Long.parseLong(fields[58]);
    if (timeUs<=0 || countsPerOutputRev<=0 || referenceId<0 || !Double.isFinite(positionRad) ||
        (valid!=0 && valid!=1) || speedWindowUs<0)
      throw new IllegalArgumentException("Encoder invalido");
    speedValid=valid==1;
    if (speedValid && (!Double.isFinite(speedRadS) || speedWindowUs==0))
      throw new IllegalArgumentException("Velocidad invalida");
    if (!speedValid) speedRadS=Double.NaN;
    supported=true;
  }
  /** Agrega campos sin duplicar los que ya contiene el CSV de escalones. */
  void addColumns(Table table) {
    for (String name:new String[]{"encoder_count_x4","encoder_version","encoder_time_us",
      "encoder_relative_count_x4","output_position_rad","output_speed_rad_s","encoder_speed_valid",
      "encoder_speed_window_us","counts_x4_per_output_rev","encoder_reference_count_x4","encoder_reference_id"})
      if (!java.util.Arrays.asList(table.getColumnTitles()).contains(name)) table.addColumn(name);
  }
  /** Exporta exactamente los valores recibidos; vacio significa no disponible. */
  void writeRow(TableRow row) {
    row.setString("encoder_count_x4",countPresent ? Long.toString(count) : "");
    row.setString("encoder_version",present ? Integer.toString(version) : "");
    row.setString("encoder_time_us",supported ? Long.toString(timeUs) : "");
    row.setString("encoder_relative_count_x4",supported ? Long.toString(relativeCount) : "");
    row.setString("output_position_rad",finiteEncoderCell(positionRad));
    row.setString("output_speed_rad_s",finiteEncoderCell(speedRadS));
    row.setString("encoder_speed_valid",supported ? (speedValid ? "1" : "0") : "");
    row.setString("encoder_speed_window_us",supported ? Long.toString(speedWindowUs) : "");
    row.setString("counts_x4_per_output_rev",supported ? Long.toString(countsPerOutputRev) : "");
    row.setString("encoder_reference_count_x4",supported ? Long.toString(referenceCount) : "");
    row.setString("encoder_reference_id",supported ? Long.toString(referenceId) : "");
  }
}

String finiteEncoderCell(double value) { return Double.isFinite(value) ? Double.toString(value) : ""; }


class SensingInfo {
  boolean present, valid, phaseValidated;
  int version, status=-1, readUs, dmaBlockSamples;
  float phaseUs=Float.NaN, pwmHz=Float.NaN, acquisitionHz=Float.NaN, adcHz=Float.NaN;
  long validCount, attempts, missed, dmaOverflows, rejectedWindow, rejectedQuality;
  float adcRawMean=Float.NaN, adcVoltageMean=Float.NaN, currentMeanA=Float.NaN;
  /** La corriente de la interfaz es exclusivamente la media calculada por ESP32. */
  float primaryCurrent(float diagnosticCurrent) { return valid ? currentMeanA : Float.NaN; }
  String primaryLabel() { return "I media DMA"; }
  SensingInfo() { }
  SensingInfo(String[] fields) {
    if (fields.length!=59 || !fields[41].equals("6"))
      throw new IllegalArgumentException("Se requiere firmware v6");
    present=true; version=6;
    int currentValid=Integer.parseInt(fields[30]);
    if (currentValid!=0 && currentValid!=1) throw new IllegalArgumentException("Validez invalida");
    valid=currentValid==1; status=Integer.parseInt(fields[31]);
    readUs=Integer.parseInt(fields[32]); phaseUs=sensingFloat(fields[33]);
    validCount=Long.parseLong(fields[34]); attempts=Long.parseLong(fields[35]); missed=Long.parseLong(fields[36]);
    pwmHz=sensingFloat(fields[37]); acquisitionHz=sensingFloat(fields[38]); adcHz=sensingFloat(fields[39]);
    dmaOverflows=Long.parseLong(fields[40]);
    rejectedWindow=Long.parseLong(fields[42]); rejectedQuality=Long.parseLong(fields[43]);
    phaseValidated=Integer.parseInt(fields[44])==1;
    adcRawMean=sensingFloat(fields[45]); adcVoltageMean=sensingFloat(fields[46]);
    currentMeanA=sensingFloat(fields[47]); dmaBlockSamples=Integer.parseInt(fields[48]);
    valid=valid && (status==9 || status==15) && dmaBlockSamples==200 &&
      Float.isFinite(adcRawMean) && Float.isFinite(adcVoltageMean) && Float.isFinite(currentMeanA);
    if (!valid) adcRawMean=adcVoltageMean=currentMeanA=Float.NaN;
  }
  boolean usable(float raw) { return valid && Float.isFinite(raw); }
  String reason() {
    if (!present) return "Esperando protocolo v6";
    if (status==9 || status==15) return valid ?
      "Media CS de 200 muestras; A equivalentes sin calibrar" : "Media DMA no valida o bloque incompleto";
    switch (status) {
      case 0: return "Esperando lectura";
      case 10: return "Perdida DMA: bloque descartado";
      case 5: return "ADC saturado";
      case 6: return "Error de lectura ADC";
      case 7: return "Error de calibracion ADC";
      default: return "Estado de corriente no valido";
    }
  }
  /** Resume conteos ADC acumulados; el CSV conserva los enteros completos. */
  String compactCount(long n) { return n>=1000000 ? nf(n/1000000.0f,0,1)+"M" : n>=1000 ? nf(n/1000.0f,0,1)+"k" : Long.toString(n); }
  String dmaDetails() { return "Bloque "+dmaBlockSamples+" | CS "+compactCount(validCount)+" | invalidas "+compactCount(rejectedQuality); }
  void addColumns(Table table) {
    for (String name:new String[]{"current_valid","current_status","current_status_text","adc_read_us","sample_phase_us",
      "current_valid_count","sample_attempt_count","sample_missed_count","pwm_frequency_observed_hz",
      "acquisition_rate_observed_hz","adc_rate_observed_hz","dma_overflows","measurement_basis","sensing_version",
      "dma_rejected_window","dma_rejected_quality","dma_phase_validated","raw_zero_assigned","adc_raw_mean",
      "adc_voltage_mean","current_mean_a","dma_block_samples"}) table.addColumn(name);
  }
  void writeRow(TableRow row) {
    row.setString("dma_overflows",Long.toString(dmaOverflows));
    row.setString("raw_zero_assigned",status==15 ? "1" : "0");
    row.setString("sensing_version",Integer.toString(version));
    row.setString("adc_raw_mean",finiteCell(adcRawMean));
    row.setString("adc_voltage_mean",finiteCell(adcVoltageMean));
    row.setString("current_mean_a",finiteCell(currentMeanA));
    row.setString("dma_block_samples",Integer.toString(dmaBlockSamples));
    row.setString("dma_rejected_window",Long.toString(rejectedWindow));
    row.setString("dma_rejected_quality",Long.toString(rejectedQuality));
    row.setString("dma_phase_validated",phaseValidated ? "1" : "0");
    row.setString("measurement_basis",valid ? "dma_block_mean_CS_raw_zero_assigned" : "unavailable");
    row.setString("current_valid",valid ? "1" : "0");
    row.setString("current_status",Integer.toString(status)); row.setString("current_status_text",reason());
    row.setString("adc_read_us",Integer.toString(readUs));
    row.setString("sample_phase_us",phaseUs>=0 ? finiteCell(phaseUs) : "");
    row.setString("current_valid_count",Long.toString(validCount));
    row.setString("sample_attempt_count",Long.toString(attempts));
    row.setString("sample_missed_count",Long.toString(missed));
    row.setString("pwm_frequency_observed_hz",finiteCell(pwmHz));
    row.setString("acquisition_rate_observed_hz",finiteCell(acquisitionHz)); row.setString("adc_rate_observed_hz",finiteCell(adcHz));
  }
}

/** Separate 20 Hz extension. Source timestamps are retained when held alongside 100 Hz DATA. */
class SpeedInfo {
  boolean supported,limited;
  long timeUs,controlTimeUs;
  int direction;
  float reference,error,currentRef,kp,ki,integral,limit;
  SpeedInfo() { }
  SpeedInfo(String[] f) {
    if (f.length!=13 || !f[1].equals("1")) throw new IllegalArgumentException("SPEED v1");
    timeUs=Long.parseLong(f[2]); direction=Integer.parseInt(f[3]);
    reference=Float.parseFloat(f[4]); error=Float.parseFloat(f[5]); currentRef=Float.parseFloat(f[6]);
    kp=Float.parseFloat(f[7]); ki=Float.parseFloat(f[8]); integral=Float.parseFloat(f[9]); limit=Float.parseFloat(f[10]);
    int flag=Integer.parseInt(f[11]); controlTimeUs=Long.parseLong(f[12]);
    for (float v:new float[]{reference,error,currentRef,kp,ki,integral,limit})
      if (!Float.isFinite(v)) throw new IllegalArgumentException("SPEED no finito");
    if (timeUs<=0 || controlTimeUs<0 || (direction!=1 && direction!=-1) || reference<0 || reference>32 ||
      kp<0 || ki<0 || limit<=0 || limit>1 || currentRef<0 || currentRef>limit+0.000001f || (flag!=0 && flag!=1))
      throw new IllegalArgumentException("SPEED invalido");
    limited=flag==1; supported=true;
  }
  String[] columns={"speed_telemetry_time_us","speed_control_time_us","speed_direction_sign","speed_ref_magnitude_rad_s",
    "speed_error_rad_s","speed_current_ref_a_eq","speed_kp","speed_ki","speed_integral_a_eq","speed_current_limit_a_eq","speed_limited"};
  void addColumns(Table t) { for (String c:columns) t.addColumn(c); }
  void writeRow(TableRow r) {
    String[] v={Long.toString(timeUs),Long.toString(controlTimeUs),Integer.toString(direction),Float.toString(reference),
      Float.toString(error),Float.toString(currentRef),Float.toString(kp),Float.toString(ki),Float.toString(integral),Float.toString(limit),limited ? "1" : "0"};
    for (int j=0;j<columns.length;j++) r.setString(columns[j],supported ? v[j] : "");
  }
}

class PositionInfo {
  boolean supported,limited;
  long timeUs,controlTimeUs,referenceId;
  double reference,error;
  float speedRef,kp,ki,integral,maxSpeed,accel,tolerance;
  int state;
  PositionInfo(){}
  PositionInfo(String[] f){
    if(f.length!=16 || !f[1].equals("1"))throw new IllegalArgumentException("POSITION v1");
    timeUs=Long.parseLong(f[2]);reference=Double.parseDouble(f[3]);error=Double.parseDouble(f[4]);
    speedRef=Float.parseFloat(f[5]);kp=Float.parseFloat(f[6]);ki=Float.parseFloat(f[7]);integral=Float.parseFloat(f[8]);
    maxSpeed=Float.parseFloat(f[9]);accel=Float.parseFloat(f[10]);tolerance=Float.parseFloat(f[11]);
    state=Integer.parseInt(f[12]);int flag=Integer.parseInt(f[13]);controlTimeUs=Long.parseLong(f[14]);referenceId=Long.parseLong(f[15]);
    for(float v:new float[]{speedRef,kp,ki,integral,maxSpeed,accel,tolerance})
      if(!Float.isFinite(v))throw new IllegalArgumentException("POSITION no finito");
    if(timeUs<=0 || controlTimeUs<0 || referenceId<0 || !Double.isFinite(reference) || !Double.isFinite(error) || Math.abs(reference)>10000 || kp<0 || ki<0 ||
      maxSpeed<=0 || maxSpeed>32 || abs(speedRef)>maxSpeed+.00001f || accel<=0 || accel>100 ||
      tolerance<.0023f || tolerance>1 || state<0 || state>3 || (flag!=0 && flag!=1))throw new IllegalArgumentException("POSITION invalido");
    limited=flag==1;supported=true;
  }
  String stateText(){return new String[]{"Siguiendo objetivo","Coast: esperando inversion","En banda: coast vigilado","Coast: llegada al objetivo"}[state];}
  String[] columns={"position_telemetry_time_us","position_ref_rad","position_error_rad","position_speed_ref_rad_s",
    "position_kp","position_ki","position_integral_rad_s","position_max_speed_rad_s","position_accel_rad_s2",
    "position_tolerance_rad","position_state","position_limited","position_control_time_us","position_reference_id"};
  void addColumns(Table t){for(String c:columns)t.addColumn(c);}
  void writeRow(TableRow row){
    String[] values={Long.toString(timeUs),Double.toString(reference),Double.toString(error),Float.toString(speedRef),
      Float.toString(kp),Float.toString(ki),Float.toString(integral),Float.toString(maxSpeed),Float.toString(accel),
      Float.toString(tolerance),Integer.toString(state),limited ? "1" : "0",Long.toString(controlTimeUs),Long.toString(referenceId)};
    for(int i=0;i<columns.length;i++)row.setString(columns[i],supported ? values[i] : "");
  }
}
