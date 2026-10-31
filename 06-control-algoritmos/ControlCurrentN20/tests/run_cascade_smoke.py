"""Short authorized hardware checks, explicit --run, raw evidence and final STOP.
Host aborts are supervisory, not independent hardware protection.
"""
import argparse,json,time,math,statistics,csv
from pathlib import Path
import serial

p=argparse.ArgumentParser()
p.add_argument('--run',action='store_true')
p.add_argument('--port',default='COM9')
p.add_argument('--stage',choices=['current','speed','position'],required=True)
args=p.parse_args()
if not args.run:raise SystemExit('Explicit --run required')
stem=Path(__file__).resolve().parents[1]/'build-codex'/('cascade-'+args.stage+'-'+time.strftime('%Y%m%d-%H%M%S'))
s=serial.Serial(port=None,baudrate=460800,timeout=.03,write_timeout=.3)
s.dtr=False;s.rts=False;s.port=args.port
log=open(str(stem)+'.jsonl','w',encoding='utf-8')
start=time.monotonic();label='baseline';last=None;extensions={};rows=[];acks=[]
overflow=None;origin=None;failure=None;monitor=True
def event(kind,line):
    log.write(json.dumps({'host_s':time.monotonic()-start,'stage':label,'kind':kind,'text':line})+'\n');log.flush()
def send(cmd):
    event('TX',cmd);s.write((cmd+'\n').encode());s.flush()
def read(seconds,mode=None):
    global last,overflow,origin
    end=time.monotonic()+seconds;rx=time.monotonic();received=[]
    while time.monotonic()<end:
        line=s.readline().decode('utf-8','replace').strip()
        if not line:
            if time.monotonic()-rx>.3:raise RuntimeError('No DATA for 300 ms')
            continue
        event('RX',line)
        if line.startswith('ERR,') and monitor:raise RuntimeError(line)
        if line.startswith('ACK,'):received.append(line);acks.append(line)
        a=line.split(',')
        if a[0] in ('SPEED','POSITION'):extensions[a[0]]=a
        if a[0]!='DATA':continue
        if len(a)!=59 or a[41]!='6':raise RuntimeError('DATA schema mismatch')
        rx=time.monotonic();last=a;rows.append([rx-start,label]+a)
        if overflow is None:overflow=int(a[40]);origin=float(a[52])
        if not monitor:continue
        if a[30]!='1' or a[48]!='200' or a[54]!='1':raise RuntimeError('Invalid ADC/encoder')
        i,pwm,w,pos=map(float,(a[47],a[18],a[53],a[52]))
        if not all(map(math.isfinite,(i,pwm,w,pos))):raise RuntimeError('Nonfinite feedback')
        if i>.15 or abs(pwm)>40.1:raise RuntimeError('CS >150 mA eq or PWM >40%')
        if abs(w)>15 or abs(pos-origin)>(1 if args.stage=='position' else 10):raise RuntimeError('Speed/travel test bound')
        if int(a[40])!=overflow:raise RuntimeError('DMA overflow changed')
        if mode and a[14]!=mode:raise RuntimeError('Unexpected mode '+a[14])
        if abs(float(a[19])-8.832)>.001 or abs(float(a[20])-80090)>.1:raise RuntimeError('Unexpected current PI')
    return received
def command(cmd,prefix):
    send(cmd)
    if not any(x.startswith(prefix) for x in read(.18)):raise RuntimeError('Missing '+prefix)
def stopped():
    global label
    label='stop';command('STOP','ACK,STOP');read(1)
    if last[14]!='OPEN' or float(last[18])!=0 or abs(float(last[53]))>.1:raise RuntimeError('Not stopped')
try:
    s.open();command('STOP','ACK,STOP');read(.5)
    if 'POSITION' not in extensions or 'SPEED' not in extensions:raise RuntimeError('Missing cascade firmware extensions')
    if abs(float(extensions['SPEED'][7])-.0284)>1e-6 or abs(float(extensions['SPEED'][8])-.01392)>1e-6:raise RuntimeError('Unexpected speed PI')
    if abs(float(extensions['POSITION'][6])-6.166)>.001 or abs(float(extensions['POSITION'][7])-3192)>.1:raise RuntimeError('Unexpected position PI')
    command('LIMIT,0,40','ACK,LIMIT')
    command('ILIMIT,0.03','ACK,ILIMIT')
    if args.stage=='current':
        command('IREF,0','ACK,IREF');command('MODE,PID,CW','ACK,MODE,PID')
        for ref in (.01,.02):
            label='current_'+str(ref);command('IREF,'+str(ref),'ACK,IREF');read(.65,'PID')
        stopped()
    elif args.stage=='speed':
        for direction in ('CW','CCW'):
            stopped();command('WREF,1','ACK,WREF')
            label='speed_'+direction;command('MODE,SPEED,'+direction,'ACK,MODE,SPEED');read(1.8,'SPEED')
            stopped()
    else:
        stopped()
        command('PLIMIT,0.5,1,0.02','ACK,PLIMIT')
        origin=float(last[52])
        for delta in (.15,-.15):
            label='position_'+str(delta);command('PREF,'+str(origin+delta),'ACK,PREF')
            command('MODE,POSITION','ACK,MODE,POSITION');read(3,'POSITION');stopped()
except Exception as exc:
    failure=str(exc);print('ABORT:',failure,flush=True)
finally:
    label='final_stop';monitor=False
    if s.is_open:
        try:send('STOP');send('IREF,0');send('WREF,0');read(1)
        except Exception as exc:event('FINAL_ERROR',str(exc))
        s.close()
    log.close()
    with open(str(stem)+'.csv','w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['host_s','stage']+['DATA_'+str(i) for i in range(59)]);w.writerows(rows)
    summary={'failure':failure,'files':str(stem),'rows':len(rows),'stop_confirmed':bool(last and last[14]=='OPEN' and float(last[18])==0 and abs(float(last[53]))<=.1),'stages':{}}
    for stage in dict.fromkeys(r[1] for r in rows):
        rr=[r for r in rows if r[1]==stage];tail=[r for r in rr if r[0]>=rr[-1][0]-.4]
        vals=lambda k,rs=tail:[float(r[k+2]) for r in rs]
        summary['stages'][stage]={'n':len(rr),'mean_CS_A_eq':statistics.mean(vals(47)), 'peak_CS_A_eq':max(vals(47,rr)), 'mean_pwm_pct':statistics.mean(vals(18)), 'peak_abs_pwm_pct':max(map(abs,vals(18,rr))), 'mean_speed_rad_s':statistics.mean(vals(53)), 'peak_abs_speed_rad_s':max(map(abs,vals(53,rr))), 'first_position_rad':vals(52,rr)[0],'last_position_rad':vals(52,rr)[-1],'dma_overflows':int(rr[-1][42])}
    Path(str(stem)+'.summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2),flush=True)
if failure:raise SystemExit(1)
