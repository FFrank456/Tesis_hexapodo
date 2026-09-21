import processing.serial.*;
import java.io.File;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

Serial esp32;
String[] ports = {};
int selectedPort, lastDataMillis, requestMillis;
String status = "Selecciona puerto y conecta", outcome = "sin ensayo", captureProgram = "";
boolean running, awaitingAck, firmwareReady;
long activeSequenceId = -1, lastSampleUs = -1, positionZero;
float captureCpr, captureVoltage = 12;
ConcurrentLinkedQueue<String> incoming = new ConcurrentLinkedQueue<String>();
volatile boolean inputOverflow;
ArrayList<Sample> samples = new ArrayList<Sample>();
Sample latest;
InputField program, countsPerRev, supplyVoltage, activeInput;
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
  countsPerRev = new InputField(28,376,208,34,"Conteos x4 / vuelta de SALIDA","0",12);
  supplyVoltage = new InputField(258,376,210,34,"Alimentacion medida (V)","12",12);
  connectButton = new Button(920,35,105,34,"Conectar");
  refreshButton = new Button(1035,35,115,34,"Refrescar");
  positiveButton = new Button(28,283,208,32,"Ejemplo positivo");
  bipolarButton = new Button(258,283,210,32,"Ejemplo ambos sentidos");
  runButton = new Button(28,444,138,42,"INICIAR");
  stopButton = new Button(178,444,138,42,"STOP");
  exportButton = new Button(328,444,140,42,"Exportar CSV");
  zeroButton = new Button(28,538,180,32,"Cero visual encoder");
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
  drawPlot(510,254,640,135,0,latest!=null && latest.sensing.version==5 ? "CS (A eq.): media DMA azul / EMA naranja" : "Corriente (A): ultima muestra aceptada / EMA");
  drawPlot(510,406,640,135,1,"PWM aplicado (%)");
  drawPlot(510,558,640,135,2,captureCpr>0 ? "Velocidad salida (RPM, ventana >= 50 ms)" : "Encoder: conteos relativos x4");
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
    status="Conectado; esperando firmware con encoder";
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
  if (!line.startsWith("DATA,")) return;
  String[] f=split(line,',');
  if (f.length<32) { firmwareReady=false; status="Actualiza firmware: faltan campos encoder/secuencia"; return; }
  Sample s;
  try { s=new Sample(f,millis()); }
  catch (RuntimeException e) { status="Trama invalida descartada"; return; }
  lastDataMillis=millis(); firmwareReady=true;
  if (latest!=null && s.timeUs<latest.timeUs && (running || awaitingAck)) stopExperiment("Abortado: cambio del reloj ESP32");
  latest=s;
  if (!running || s.sequenceId!=activeSequenceId || s.timeUs<=lastSampleUs) return;
  if (samples.size()>=20000) { stopExperiment("Abortado: limite de muestras"); return; }
  if (samples.isEmpty()) positionZero=s.encoderCount;
  s.elapsedUs=s.timeUs-s.startUs;
  if (captureCpr>0 && !samples.isEmpty()) {
    int j=samples.size()-1;
    while (j>0 && s.timeUs-samples.get(j).timeUs<50000) j--;
    Sample before=samples.get(j);
    long dt=s.timeUs-before.timeUs;
    if (dt>=50000) s.rpm=(float)((s.encoderCount-before.encoderCount)*60000000.0/(captureCpr*dt));
  }
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
    float cpr=Float.parseFloat(countsPerRev.value), volts=Float.parseFloat(supplyVoltage.value);
    if (!Float.isFinite(cpr) || cpr<0 || !Float.isFinite(volts) || volts<=0) throw new IllegalArgumentException("Conteos >= 0 y voltaje > 0");
    captureProgram=spec; captureCpr=cpr; captureVoltage=volts;
    samples.clear(); activeSequenceId=-1; lastSampleUs=-1; outcome="en curso";
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

void mousePressed() {
  activeInput=null;
  if (stopButton.contains(mouseX,mouseY)) { stopExperiment("Detenido por usuario"); return; }
  if (running || awaitingAck) return;
  if (program.contains(mouseX,mouseY)) activeInput=program;
  else if (countsPerRev.contains(mouseX,mouseY)) activeInput=countsPerRev;
  else if (supplyVoltage.contains(mouseX,mouseY)) activeInput=supplyVoltage;
  else if (connectButton.contains(mouseX,mouseY)) connectSelectedPort();
  else if (refreshButton.contains(mouseX,mouseY)) refreshPorts();
  else if (runButton.contains(mouseX,mouseY)) startExperiment();
  else if (exportButton.contains(mouseX,mouseY)) exportExperiment();
  else if (positiveButton.contains(mouseX,mouseY)) program.value=rampProgram();
  else if (bipolarButton.contains(mouseX,mouseY)) program.value="0:1000;20:2000;0:1500;-20:2000;0:1500";
  else if (zeroButton.contains(mouseX,mouseY) && latest!=null) positionZero=latest.encoderCount;
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

Table experimentTable() {
  Table table=new Table();
  String[] columns={"pc_receive_ms","sample_time_us","relative_time_us","adc_sample_count","adc_raw","adc_voltage",
    "current_raw_a","current_ema_a","pwm_applied_percent","encoder_count_x4","encoder_relative_count_x4",
    "output_position_rev","output_speed_rpm","counts_x4_per_output_rev","sequence_id","stage_index",
    "sequence_start_us","stage_start_us","supply_voltage_user_v","estimated_armature_voltage_v","program","outcome"};
  for (String column:columns) table.addColumn(column);
  new SensingInfo().addColumns(table);
  table.addColumn("pwm_frequency_nominal_hz"); table.addColumn("acquisition_rate_nominal_hz");
  long firstCount=samples.get(0).encoderCount;
  for (Sample s:samples) {
    String[] values={Long.toString(s.pcMs),Long.toString(s.timeUs),Long.toString(s.elapsedUs),Long.toString(s.adcSampleCount),
      s.raw>=0 ? Integer.toString(s.raw) : "",finiteCell(s.voltage),finiteCell(s.current),finiteCell(s.ema),Float.toString(s.pwm),
      Long.toString(s.encoderCount),Long.toString(s.encoderCount-firstCount),
      captureCpr>0 ? Double.toString((s.encoderCount-firstCount)/(double)captureCpr) : "",
      Float.isFinite(s.rpm) ? Float.toString(s.rpm) : "",Float.toString(captureCpr),Long.toString(s.sequenceId),
      Integer.toString(s.stage),Long.toString(s.startUs),Long.toString(s.stageStartUs),Float.toString(captureVoltage),
      Float.toString(s.pwm*captureVoltage/100),captureProgram,outcome};
    TableRow row=table.addRow();
    for (int i=0;i<columns.length;i++) row.setString(columns[i],values[i]);
    s.sensing.writeRow(row);
    row.setString("pwm_frequency_nominal_hz",finiteCell(s.nominalPwmHz));
    row.setString("acquisition_rate_nominal_hz",finiteCell(s.nominalAcquisitionHz));
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
  countsPerRev.draw(); supplyVoltage.draw();
  fill(165); text("0 = sin calibrar: se guardan conteos, sin inventar RPM.",28,432);
  runButton.draw(); stopButton.draw(); exportButton.draw();
  fill(220); textSize(15); text("Muestras guardadas: "+samples.size(),28,518); zeroButton.draw();
  fill(175); textSize(13);
  text("Encoder A: GPIO18 | B: GPIO19 | conteo x4\nEncoder antes de reductora 100:1.\nCPR salida = CPR rotor x 100 (ya contado en x4).\n\nEnter: terminar edicion. Delete: vaciar campo.\nEl ESP32 ejecuta y termina la secuencia por tiempo.\nRegistro aprox. 100 Hz; usa los tiempos ESP32.",28,588,444,115);
}

void drawMeasurements() {
  fill(230); textSize(17); text(running ? "ENSAYO EN CURSO" : awaitingAck ? "Esperando confirmacion" : "Mediciones",510,115);
  if (latest==null) return;
  fill(210); textSize(14);
  text(latest.sensing.primaryLabel()+": "+currentText(latest.sensing.primaryCurrent(latest.current))+"    EMA: "+currentText(latest.ema),510,138);
  fill(latest.sensing.valid ? color(130,220,175) : color(240,190,100)); textSize(12);
  text(latest.sensing.reason(),510,157); fill(210); textSize(14);
  text("PWM: "+nf(latest.pwm,1,2)+" %    Etapa: "+latest.stage,510,178);
  text("Encoder x4: "+latest.encoderCount+"    Relativo: "+(latest.encoderCount-positionZero),510,199);
  textSize(12);
  text(latest.sensing.version>=2 ? latest.sensing.dmaDetails()+(latest.sensing.version==5 ? " | V media: "+finiteCell(latest.sensing.adcVoltageMean) : " | V SDK: "+finiteCell(latest.voltage)) :
    "ADC: "+latest.raw+" | inicio <= "+finiteCell(latest.sensing.phaseUs)+" us | lectura "+latest.sensing.readUs+" us",510,219);
  text("Obs.: PWM "+rateText(latest.sensing.pwmHz)+" Hz | Adq. "+rateText(latest.sensing.acquisitionHz)+
    " Hz | ADC nom. "+rateText(latest.nominalAcquisitionHz)+" Hz | ADC "+rateText(latest.sensing.adcHz)+" Hz",510,240);
}

void drawPlot(float x,float y,float w,float h,int signal,String title) {
  noStroke(); fill(29); rect(x,y,w,h,6); fill(210); textSize(13); text(title,x+12,y+20);
  if (samples.size()<2) return;
  long tmax=Math.max(1L,samples.get(samples.size()-1).elapsedUs);
  float low=signal==1 ? -100 : 0, high=signal==1 ? 100 : signal==0 ? 0.1 : 1;
  for (Sample s:samples) {
    float v=plotValue(s,signal);
    if (Float.isFinite(v)) { low=min(low,v); high=max(high,v); }
    if (signal==0 && Float.isFinite(s.ema)) high=max(high,s.ema);
  }
  if (high-low<0.001) high=low+1;
  fill(150); textSize(11); text(nf(high,1,signal==0 ? 3 : 1),x+6,y+42);
  text(nf(low,1,signal==0 ? 3 : 1),x+6,y+h-13); text(nf(tmax/1000000.0f,1,2)+" s",x+w-60,y+h-5);
  for (int trace=0;trace<(signal==0 ? 2 : 1);trace++) {
    stroke(trace==1 ? color(255,130,105) : color(90,185,255)); noFill();
    boolean segment=false;
    // Examine every row so a short invalid interval cannot disappear in decimation.
    for (int j=0;j<samples.size();j++) {
      Sample s=samples.get(j); float value=trace==1 ? s.ema : plotValue(s,signal);
      if (!Float.isFinite(value)) { if (segment) endShape(); segment=false; continue; }
      if (!segment) { beginShape(); segment=true; }
      vertex(x+58+(float)(s.elapsedUs/(double)tmax)*(w-72),map(value,low,high,y+h-22,y+35));
    }
    if (segment) endShape();
  }
}

float plotValue(Sample s,int signal) {
  if (signal==0) return s.sensing.primaryCurrent(s.current);
  if (signal==1) return s.pwm;
  return captureCpr>0 ? s.rpm : (float)(s.encoderCount-samples.get(0).encoderCount);
}

class Sample {
  long pcMs,timeUs,elapsedUs,adcSampleCount,encoderCount,sequenceId,startUs,stageStartUs;
  int raw,stage;
  float voltage,current,ema,pwm,rpm=Float.NaN,nominalPwmHz,nominalAcquisitionHz;
  SensingInfo sensing;
  Sample(String[] f,long pc) {
    pcMs=pc; raw=Integer.parseInt(f[2]); voltage=sensingFloat(f[3]); current=sensingFloat(f[4]); ema=sensingFloat(f[5]);
    sensing=new SensingInfo(f);
    if (!sensing.usable(current,ema)) { current=ema=Float.NaN; sensing.valid=false; }
    nominalPwmHz=sensingFloat(f[11]); nominalAcquisitionHz=sensingFloat(f[12]);
    adcSampleCount=Long.parseLong(f[10]); pwm=Float.parseFloat(f[20]); timeUs=Long.parseLong(f[26]); encoderCount=Long.parseLong(f[27]);
    sequenceId=Long.parseLong(f[28]); stage=Integer.parseInt(f[29]); startUs=Long.parseLong(f[30]); stageStartUs=Long.parseLong(f[31]);
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
