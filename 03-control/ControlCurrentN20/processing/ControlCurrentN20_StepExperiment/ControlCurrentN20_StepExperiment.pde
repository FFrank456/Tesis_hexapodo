import processing.serial.*;
import java.io.File;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

Serial esp32;
String[] ports = {};
int selectedPort, lastDataMillis, requestMillis;
String status = "Selecciona puerto y conecta", outcome = "sin ensayo", captureProgram = "";
boolean running, awaitingAck, firmwareReady;
long activeSequenceId = -1, lastSampleUs = -1, captureReferenceId = -1;
float captureVoltage = 12;
ConcurrentLinkedQueue<String> incoming = new ConcurrentLinkedQueue<String>();
volatile boolean inputOverflow;
ArrayList<Sample> samples = new ArrayList<Sample>();
Sample latest;
InputField program, supplyVoltage, activeInput;
Button connectButton, refreshButton, runButton, stopButton, exportButton, positiveButton, bipolarButton, zeroButton;

void setup() {
  size(1180, 760);
  surface.setTitle("N20 - Escalones PWM y encoder");
  textFont(createFont("Arial", 15));
  initControls();
  refreshPorts();
}

void initControls() {
  program = new InputField(28,164,440,104,"Niveles: PWM % : duracion ms; ...",rampProgram(),480);
  supplyVoltage = new InputField(258,376,210,34,"Alimentacion medida (V)","12",12);
  connectButton = new Button(920,35,105,34,"Conectar");
  refreshButton = new Button(1035,35,115,34,"Refrescar");
  positiveButton = new Button(28,283,208,32,"Ejemplo positivo");
  bipolarButton = new Button(258,283,210,32,"Ejemplo ambos sentidos");
  runButton = new Button(28,444,138,42,"INICIAR");
  stopButton = new Button(178,444,138,42,"STOP");
  exportButton = new Button(328,444,140,42,"Exportar CSV");
  zeroButton = new Button(28,538,200,32,"Cero encoder ESP32");
}

// Rampa de identificacion: 0 -> 100 % y regreso a 0 %, en pasos de 5 %.
// Cada nivel dura un segundo. Se mantiene el signo positivo para caracterizar
// el sentido horario; el boton de ambos sentidos sigue disponible aparte.
String rampProgram() {
  String result="";
  for (int pwm=0; pwm<=100; pwm+=5) {
    if (!result.isEmpty()) result+=";";
    result+=pwm+":1000";
  }
  for (int pwm=95; pwm>=0; pwm-=5) result+=";"+pwm+":1000";
  return result;
}

void draw() {
  background(18);
  while (!incoming.isEmpty()) processLine(incoming.poll());
  if (inputOverflow) { inputOverflow=false; stopExperiment("Abortado: desbordamiento de recepcion"); }
  if ((running || awaitingAck) && millis()-lastDataMillis > 1500) stopExperiment("Abortado: sin telemetria");
  if (awaitingAck && millis()-requestMillis > 2500) stopExperiment("Abortado: falta ACK de secuencia");
  drawControls();
  drawMeasurements();
  drawPlot(510,264,640,125,0,"CS (A eq.): media DMA de 200 muestras");
  drawPlot(510,406,640,135,1,"PWM aplicado (%)");
  drawPlot(510,558,640,135,2,"Velocidad salida (rad/s; calculada por ESP32)");
  fill(180); textSize(13); text(status,28,715,width-56,38);
}

void refreshPorts() {
  ports=Serial.list(); selectedPort=constrain(selectedPort,0,max(0,ports.length-1));
}

void connectSelectedPort() {
  if (running || awaitingAck) { status="Deten el ensayo antes de reconectar"; return; }
  if (ports.length==0) { status="No hay puertos"; return; }
  if (esp32!=null) esp32.stop();
  esp32=null; incoming.clear(); firmwareReady=false; latest=null;
  try {
    esp32=new Serial(this,ports[selectedPort],460800);
    esp32.clear(); esp32.bufferUntil('\n'); lastDataMillis=millis();
    status="Conectado; esperando firmware v6 con encoder";
  } catch (RuntimeException e) { status="No se pudo abrir el puerto: "+e.getMessage(); }
}

void serialEvent(Serial port) {
  String line=port.readStringUntil('\n');
  if (line==null) return;
  if (incoming.size()>=4096) { inputOverflow=true; return; }
  incoming.add(line.trim());
}

void processLine(String line) {
  if (line==null) return;
  if (line.startsWith("HEADER,")) {
    if (running || awaitingAck) stopExperiment("Abortado: reinicio del ESP32");
    firmwareReady=false; latest=null; return;
  }
  if (line.startsWith("ERR,")) {
    if (awaitingAck) { awaitingAck=false; outcome="rechazado"; }
    status=line; return;
  }
  if (line.startsWith("ACK,SEQ,")) {
    if (!awaitingAck) return;
    try { activeSequenceId=Long.parseLong(split(line,',')[2]); }
    catch (RuntimeException e) { stopExperiment("ACK invalido"); return; }
    awaitingAck=false; running=true; status="Secuencia confirmada por ESP32"; return;
  }
  if (line.equals("ACK,ENCODER,ZERO")) {
    status="Cero del encoder confirmado por ESP32"; return;
  }
  if (!line.startsWith("DATA,")) return;
  String[] f=split(line,',');
  if (f.length!=59 || !f[41].equals("6")) {
    if (running || awaitingAck) stopExperiment("Abortado: protocolo incompatible");
    firmwareReady=false; latest=null;
    status="Actualiza firmware: se requiere protocolo v6 (59 campos)"; return;
  }
  Sample s;
  try { s=new Sample(f,millis()); }
  catch (RuntimeException e) { status="Trama invalida descartada"; return; }
  lastDataMillis=millis(); firmwareReady=true;
  if (latest!=null && s.timeUs<latest.timeUs && (running || awaitingAck)) stopExperiment("Abortado: cambio del reloj ESP32");
  latest=s;
  if (!running || s.sequenceId!=activeSequenceId || s.timeUs<=lastSampleUs) return;
  if (samples.size()>=20000) { stopExperiment("Abortado: limite de muestras"); return; }
  if (captureReferenceId>=0 && (!s.encoder.supported || s.encoder.referenceId!=captureReferenceId)) {
    stopExperiment("Abortado: cambio de referencia del encoder ESP32"); return;
  }
  if (s.encoder.supported) captureReferenceId=s.encoder.referenceId;
  s.elapsedUs=s.timeUs-s.startUs;
  samples.add(s); lastSampleUs=s.timeUs;
  if (s.stage<0) {
    running=false;
    outcome=s.elapsedUs>=programDurationUs(captureProgram) ? "terminado" : "interrumpido";
    status="Ensayo "+outcome+". PWM detenido; exporta el CSV.";
  }
}

String validatedProgram(String input) {
  String[] stages=input.replaceAll("\\s+","").split(";",-1);
  if (stages.length<1 || stages.length>48) throw new IllegalArgumentException("Usa 1 a 48 niveles");
  long total=0; String result="";
  for (String stage:stages) {
    String[] pair=stage.split(":",-1);
    if (pair.length!=2) throw new IllegalArgumentException("Formato: 20:2000;40:2000;0:1000");
    float pwm=Float.parseFloat(pair[0]); int ms=Integer.parseInt(pair[1]);
    if (!Float.isFinite(pwm) || pwm< -100 || pwm>100) throw new IllegalArgumentException("PWM: -100 a 100 %");
    if (ms<100 || ms>30000) throw new IllegalArgumentException("Cada nivel: 100 a 30000 ms");
    total+=ms;
    if (!result.isEmpty()) result+=";";
    result+=Float.toString(pwm)+":"+ms;
  }
  if (total>120000 || result.length()+4>=512) throw new IllegalArgumentException("Maximo 120 s / 511 caracteres");
  return result;
}

long programDurationUs(String spec) {
  long total=0;
  for (String stage:spec.split(";")) total+=Long.parseLong(stage.split(":")[1])*1000;
  return total;
}

void startExperiment() {
  if (running || awaitingAck) return;
  if (esp32==null || !firmwareReady || millis()-lastDataMillis>1500) { status="Conecta y espera telemetria del firmware actualizado"; return; }
  try {
    String spec=validatedProgram(program.value);
    float volts=Float.parseFloat(supplyVoltage.value);
    if (!Float.isFinite(volts) || volts<=0) throw new IllegalArgumentException("Voltaje > 0");
    captureProgram=spec; captureVoltage=volts;
    samples.clear(); activeSequenceId=-1; lastSampleUs=-1; captureReferenceId=-1; outcome="en curso";
    requestMillis=millis(); awaitingAck=true;
    sendCommand("SEQ,"+spec);
  } catch (RuntimeException e) { status="Revisa los valores: "+e.getMessage(); }
}

void sendCommand(String command) {
  if (esp32==null) return;
  try { esp32.write(command+"\n"); }
  catch (RuntimeException e) {
    running=awaitingAck=false; outcome="error serial";
    status="Error serial; la secuencia ESP32 termina por su temporizador";
  }
}

void stopExperiment(String reason) {
  sendCommand("STOP"); running=awaitingAck=false; outcome="interrumpido"; status=reason;
}

/** Solicita al firmware cambiar la referencia; la interfaz nunca modifica conteos ni angulos. */
void zeroEncoder() {
  if (running || awaitingAck) { status="Deten el ensayo antes de cambiar el cero"; return; }
  if (latest==null || !latest.encoder.supported) { status="Actualiza firmware para poner a cero el encoder"; return; }
  sendCommand("ENCODER,ZERO");
}

void mousePressed() {
  activeInput=null;
  if (stopButton.contains(mouseX,mouseY)) { stopExperiment("Detenido por usuario"); return; }
  if (running || awaitingAck) return;
  if (program.contains(mouseX,mouseY)) activeInput=program;
  else if (supplyVoltage.contains(mouseX,mouseY)) activeInput=supplyVoltage;
  else if (connectButton.contains(mouseX,mouseY)) connectSelectedPort();
  else if (refreshButton.contains(mouseX,mouseY)) refreshPorts();
  else if (runButton.contains(mouseX,mouseY)) startExperiment();
  else if (exportButton.contains(mouseX,mouseY)) exportExperiment();
  else if (positiveButton.contains(mouseX,mouseY)) program.value=rampProgram();
  else if (bipolarButton.contains(mouseX,mouseY)) program.value="0:1000;20:2000;0:1500;-20:2000;0:1500";
  else if (zeroButton.contains(mouseX,mouseY)) zeroEncoder();
}

void keyPressed() {
  if (key==' ') { stopExperiment("Detenido por usuario"); return; }
  if (activeInput!=null) { activeInput.handleKey(); return; }
  if (key=='s' || key=='S') startExperiment();
  else if (key=='e' || key=='E') exportExperiment();
  else if (key==ENTER || key==RETURN) connectSelectedPort();
  else if (key=='r' || key=='R') refreshPorts();
  if (!running && !awaitingAck) {
    if (keyCode==UP) selectedPort=max(0,selectedPort-1);
    if (keyCode==DOWN) selectedPort=min(max(0,ports.length-1),selectedPort+1);
  }
}

void dispose() {
  if (esp32!=null) {
    try { esp32.write("STOP\n"); esp32.stop(); } catch (RuntimeException e) { }
  }
  super.dispose();
}

/** Exporta rad, rad/s y referencia recibidos del ESP32, sin conversiones ni derivadas locales. */
Table experimentTable() {
  Table table=new Table();
  String[] columns={"pc_receive_ms","sample_time_us","relative_time_us","adc_sample_count","adc_raw","adc_voltage",
    "current_raw_a","pwm_applied_percent","encoder_count_x4","encoder_relative_count_x4",
    "output_position_rad","output_speed_rad_s","counts_x4_per_output_rev","sequence_id","stage_index",
    "sequence_start_us","stage_start_us","supply_voltage_user_v","estimated_armature_voltage_v","program","outcome"};
  for (String column:columns) table.addColumn(column);
  new SensingInfo().addColumns(table);
  table.addColumn("pwm_frequency_nominal_hz"); table.addColumn("acquisition_rate_nominal_hz");
  new EncoderInfo().addColumns(table);
  for (Sample s:samples) {
    String[] values={Long.toString(s.pcMs),Long.toString(s.timeUs),Long.toString(s.elapsedUs),Long.toString(s.adcSampleCount),
      s.raw>=0 ? Integer.toString(s.raw) : "",finiteCell(s.voltage),finiteCell(s.current),Float.toString(s.pwm),
      Long.toString(s.encoderCount),s.encoder.supported ? Long.toString(s.encoder.relativeCount) : "",
      finiteEncoderCell(s.encoder.positionRad),finiteEncoderCell(s.encoder.speedRadS),
      s.encoder.supported ? Long.toString(s.encoder.countsPerOutputRev) : "",Long.toString(s.sequenceId),
      Integer.toString(s.stage),Long.toString(s.startUs),Long.toString(s.stageStartUs),Float.toString(captureVoltage),
      Float.toString(s.pwm*captureVoltage/100),captureProgram,outcome};
    TableRow row=table.addRow();
    for (int i=0;i<columns.length;i++) row.setString(columns[i],values[i]);
    s.sensing.writeRow(row);
    row.setString("pwm_frequency_nominal_hz",finiteCell(s.nominalPwmHz));
    row.setString("acquisition_rate_nominal_hz",finiteCell(s.nominalAcquisitionHz));
    s.encoder.writeRow(row);
  }
  return table;
}

void exportExperiment() {
  if (running || awaitingAck) { status="Deten el ensayo antes de exportar"; return; }
  if (samples.isEmpty()) { status="No hay muestras"; return; }
  String name="multistep_"+year()+nf(month(),2)+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2)+".csv";
  File output=new File(sketchPath("exports"),name); output.getParentFile().mkdirs();
  try { saveTable(experimentTable(),output.getAbsolutePath()); status="Exportado: "+output.getAbsolutePath(); }
  catch (RuntimeException e) { status="No se pudo exportar: "+e.getMessage(); }
}

void drawControls() {
  fill(240); textSize(24); text("N20 | Identificacion con escalones",28,40);
  fill(170); textSize(13); text("S: iniciar   Espacio: STOP   E: exportar   Flechas: puerto",28,70);
  text(ports.length==0 ? "Sin puertos" : ports[selectedPort],720,57);
  connectButton.draw(); refreshButton.draw();
  fill(225); textSize(18); text("Programa de PWM",28,120);
  program.draw(); positiveButton.draw(); bipolarButton.draw();
  fill(175); textSize(12); text("+ horario / - antihorario. Cada nivel: 100-30000 ms; total <= 120 s.",28,338);
  supplyVoltage.draw();
  fill(190); textSize(12); text("Conteos x4 / vuelta de SALIDA",28,366);
  fill(35); stroke(70); rect(28,376,208,34,5); fill(235); textSize(14);
  text(latest!=null && latest.encoder.supported ? Long.toString(latest.encoder.countsPerOutputRev) : "Sin dato del ESP32",36,399);
  fill(165); textSize(12); text("Calibracion y calculos del encoder: ESP32.",28,432);
  runButton.draw(); stopButton.draw(); exportButton.draw();
  fill(220); textSize(15); text("Muestras guardadas: "+samples.size(),28,518); zeroButton.draw();
  fill(175); textSize(13);
  text("Encoder A: GPIO35 | B: GPIO25 | conteo x4\nPosicion: rad | velocidad: rad/s, recibidas del ESP32.\nCada secuencia fija su referencia en el firmware.\n\nEnter: terminar edicion. Delete: vaciar campo.\nEl ESP32 ejecuta y termina la secuencia por tiempo.\nRegistro aprox. 100 Hz; usa los tiempos ESP32.",28,588,444,115);
}

void drawMeasurements() {
  fill(230); textSize(17); text(running ? "ENSAYO EN CURSO" : awaitingAck ? "Esperando confirmacion" : "Mediciones",510,115);
  if (latest==null) return;
  fill(210); textSize(14);
  text(latest.sensing.primaryLabel()+": "+currentText(latest.sensing.primaryCurrent(latest.current)),510,138);
  fill(latest.sensing.valid ? color(130,220,175) : color(240,190,100)); textSize(12);
  text(latest.sensing.reason(),510,157); fill(210); textSize(14);
  text("PWM: "+nf(latest.pwm,1,2)+" %    Etapa: "+latest.stage,510,178);
  text("Pos. salida: "+angularText(latest.encoder.positionRad,"rad")+
    "    Vel.: "+angularText(latest.encoder.speedRadS,"rad/s"),510,199);
  textSize(12);
  text("Encoder x4: "+latest.encoderCount+"    Relativo: "+
    (latest.encoder.supported ? Long.toString(latest.encoder.relativeCount)+" | ventana "+latest.encoder.speedWindowUs+" us" : "sin datos SI del ESP32"),510,217);
  text(latest.sensing.dmaDetails()+" | V media: "+finiteCell(latest.sensing.adcVoltageMean),510,235);
  text("Obs.: PWM "+rateText(latest.sensing.pwmHz)+" Hz | Adq. "+rateText(latest.sensing.acquisitionHz)+
    " Hz | ADC nom. "+rateText(latest.nominalAcquisitionHz)+" Hz | ADC "+rateText(latest.sensing.adcHz)+" Hz",510,253);
}

/** Formato SI para pantalla; los datos desconocidos conservan su estado no disponible. */
String angularText(double value,String unit) {
  return Double.isFinite(value) ? String.format(java.util.Locale.US,"%.3f %s",value,unit) : "sin lectura";
}

void drawPlot(float x,float y,float w,float h,int signal,String title) {
  noStroke(); fill(29); rect(x,y,w,h,6); fill(210); textSize(13); text(title,x+12,y+20);
  if (samples.size()<2) return;
  long tmax=Math.max(1L,samples.get(samples.size()-1).elapsedUs);
  float low=signal==1 ? -100 : 0, high=signal==1 ? 100 : signal==0 ? 0.1 : 1;
  for (Sample s:samples) {
    float v=plotValue(s,signal);
    if (Float.isFinite(v)) { low=min(low,v); high=max(high,v); }
  }
  if (high-low<0.001) high=low+1;
  fill(150); textSize(11); text(nf(high,1,signal==0 ? 3 : 1),x+6,y+42);
  text(nf(low,1,signal==0 ? 3 : 1),x+6,y+h-13); text(nf(tmax/1000000.0f,1,2)+" s",x+w-60,y+h-5);
  stroke(color(90,185,255)); noFill();
  boolean segment=false;
  // Examine every row so a short invalid interval cannot disappear in decimation.
  for (int j=0;j<samples.size();j++) {
    Sample s=samples.get(j); float value=plotValue(s,signal);
    if (!Float.isFinite(value)) { if (segment) endShape(); segment=false; continue; }
    if (!segment) { beginShape(); segment=true; }
    vertex(x+58+(float)(s.elapsedUs/(double)tmax)*(w-72),map(value,low,high,y+h-22,y+35));
  }
  if (segment) endShape();
}

float plotValue(Sample s,int signal) {
  if (signal==0) return s.sensing.primaryCurrent(s.current);
  if (signal==1) return s.pwm;
  return (float)s.encoder.speedRadS;
}

class Sample {
  long pcMs,timeUs,elapsedUs,adcSampleCount,encoderCount,sequenceId,startUs,stageStartUs;
  int raw,stage;
  float voltage,current,pwm,nominalPwmHz,nominalAcquisitionHz;
  SensingInfo sensing;
  EncoderInfo encoder;
  Sample(String[] f,long pc) {
    pcMs=pc; raw=Integer.parseInt(f[2]); voltage=sensingFloat(f[3]); current=sensingFloat(f[4]);
    sensing=new SensingInfo(f);
    if (!sensing.usable(current)) current=Float.NaN;
    nominalPwmHz=sensingFloat(f[10]); nominalAcquisitionHz=sensingFloat(f[11]);
    adcSampleCount=Long.parseLong(f[9]); pwm=Float.parseFloat(f[18]); timeUs=Long.parseLong(f[24]); encoderCount=Long.parseLong(f[25]);
    sequenceId=Long.parseLong(f[26]); stage=Integer.parseInt(f[27]); startUs=Long.parseLong(f[28]); stageStartUs=Long.parseLong(f[29]);
    encoder=new EncoderInfo(f);
    if (!Float.isFinite(pwm)) throw new IllegalArgumentException();
  }
}

class Button {
  float x,y,w,h; String label;
  Button(float x,float y,float w,float h,String label) { this.x=x; this.y=y; this.w=w; this.h=h; this.label=label; }
  boolean contains(float px,float py) { return px>=x && px<=x+w && py>=y && py<=y+h; }
  void draw() {
    noStroke(); fill(label.equals("STOP") ? color(190,55,55) : contains(mouseX,mouseY) ? color(65) : color(43));
    rect(x,y,w,h,5); fill(235); textSize(14); textAlign(CENTER,CENTER); text(label,x+w/2,y+h/2); textAlign(LEFT,BASELINE);
  }
}

class InputField {
  float x,y,w,h; String label,value; int limit;
  InputField(float x,float y,float w,float h,String label,String value,int limit) {
    this.x=x; this.y=y; this.w=w; this.h=h; this.label=label; this.value=value; this.limit=limit;
  }
  boolean contains(float px,float py) { return px>=x && px<=x+w && py>=y && py<=y+h; }
  void handleKey() {
    if (key==ENTER || key==RETURN) { activeInput=null; return; }
    if (key==ESC) { key=0; activeInput=null; return; }
    if (key==DELETE) { value=""; return; }
    if (key==BACKSPACE) { if (!value.isEmpty()) value=value.substring(0,value.length()-1); return; }
    if (value.length()<limit && ((key>='0' && key<='9') || "+-.:;".indexOf(key)>=0)) value+=key;
  }
  void draw() {
    fill(190); textSize(12); text(label,x,y-10);
    stroke(activeInput==this ? color(80,180,255) : color(70)); fill(35); rect(x,y,w,h,5);
    String shown=value.replace(";", "; ");
    // Keep the edited tail visible even for the longest supported program.
    if (shown.length()>180) shown="..."+shown.substring(shown.length()-180);
    fill(235); textSize(14); text(shown+(activeInput==this ? "|" : ""),x+8,y+6,w-16,h-8);
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
