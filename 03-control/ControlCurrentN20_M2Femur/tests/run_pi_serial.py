"""Bounded, attended PI trial. Executes hardware commands only with --run.
Logs raw serial lines and host timestamps; always requests STOP in finally.
"""
import argparse, csv, json, math, statistics, time, runpy
from pathlib import Path
import serial

p=argparse.ArgumentParser(); p.add_argument('--run',action='store_true'); p.add_argument('--port',default='COM6')
args=p.parse_args()
if not args.run: raise SystemExit('Use --run for the authorized hardware trial.')
root=Path(__file__).resolve().parents[1]
columns=runpy.run_path(str(root/'tests/test_serial_schema.py'))['expected']
columns=['record']+columns[1:]
stem=root/'build-codex'/('pi-trial-'+time.strftime('%Y%m%d-%H%M%S'))
s=serial.Serial(port=None,baudrate=460800,timeout=.05,write_timeout=.5)
s.dtr=False; s.rts=False; s.port=args.port
events=[]; rows=[]; label='baseline'; start=time.monotonic(); last=None; overflow=None
raw=open(str(stem)+'.jsonl','w',encoding='utf8')
def log(kind,text):
    entry={'host_s':time.monotonic()-start,'stage':label,'kind':kind,'text':text}
    raw.write(json.dumps(entry)+'\n');raw.flush()
    return entry
def send(command):
    log('TX',command);s.write((command+'\n').encode());s.flush()
def read_for(seconds,active=False):
    global last,overflow
    until=time.monotonic()+seconds; last_rx=time.monotonic(); got=[]
    while time.monotonic()<until:
        line=s.readline().decode('utf8','replace').strip()
        if not line:
            if active and time.monotonic()-last_rx>.3: raise RuntimeError('Telemetry lost >300 ms')
            continue
        event=log('RX',line)
        if line.startswith('ERR,'): raise RuntimeError(line)
        if not line.startswith('DATA,'):
            if line.startswith('ACK,'): got.append(line); events.append(line)
            continue
        a=line.split(',')
        if len(a)!=59: raise RuntimeError('Incomplete DATA')
        last_rx=time.monotonic();last=a;rows.append([event['host_s'],label]+a)
        if a[41]!='6': raise RuntimeError('Not protocol v6')
        if overflow is None: overflow=int(a[40])
        if active:
            if a[14]!='PID': raise RuntimeError('PI exited: status '+a[31])
            if a[30]!='1' or a[48]!='200': raise RuntimeError('Invalid sensing: '+a[31])
            current=float(a[47]); duty=abs(float(a[18]))
            if not math.isfinite(current) or current>.15: raise RuntimeError('CS exceeds trial threshold 0.15 A eq')
            if duty>40.1 or int(a[40])!=overflow: raise RuntimeError('PWM ceiling / DMA fault')
            if abs(float(a[19])-504.9)>.01 or abs(float(a[20])-7462)>.1: raise RuntimeError('Unexpected gains')
    return got
def command(cmd,ack):
    send(cmd);responses=read_for(.15)
    if not any(x.startswith(ack) for x in responses): raise RuntimeError('Missing '+ack)

failure=None
try:
    s.open(); command('STOP','ACK,STOP');command('IREF,0','ACK,IREF');read_for(.4)
    command('PID,504.9,7462,0','ACK,PID')
    command('LIMIT,0,40','ACK,LIMIT')
    command('MODE,PID,CW','ACK,MODE')
    read_for(.3,True)
    for label,ref in [('ref_10mA',.01),('ref_20mA',.02),('ref_40mA',.04),('ref_20mA_down',.02),('ref_zero',0)]:
        send('IREF,'+str(ref));read_for(1.2,True)
    label='stop';send('STOP');send('IREF,0');read_for(.5)
except Exception as exc:
    failure=str(exc);print('TRIAL ABORT:',failure,flush=True)
finally:
    label='final_stop'
    if s.is_open:
        try: send('STOP');send('IREF,0');read_for(.5)
        except Exception as exc: print('STOP verification error:',exc,flush=True)
        s.close()
    raw.close()
    with open(str(stem)+'.csv','w',newline='') as f:
        w=csv.writer(f);w.writerow(['host_s','stage']+columns);w.writerows(rows)
    summary={'failure':failure,'rows':len(rows),'events':events,'stages':{},'stop_confirmed':bool(last and last[14]=='OPEN' and float(last[18])==0 and float(last[15])==0),'files':str(stem)}
    for stage in dict.fromkeys(r[1] for r in rows):
        rr=[r for r in rows if r[1]==stage]; good=[r for r in rr if r[32]=='1' and math.isfinite(float(r[49]))]
        if not good: continue
        tail=[r for r in good if float(r[0])>=float(good[-1][0])-.4]
        val=lambda col:[float(r[col+2]) for r in tail]
        summary['stages'][stage]={'n':len(rr),'ref':val(15)[-1],'mean_A_eq':statistics.mean(val(47)),'std_A_eq':statistics.pstdev(val(47)), 'peak_A_eq':max(float(r[49]) for r in good),'mean_pwm_pct':statistics.mean(val(18)),'min_pwm_pct':min(val(18)),'max_pwm_pct':max(val(18)), 'mean_speed_rad_s':statistics.mean(val(53)),'count_delta':int(rr[-1][27])-int(rr[0][27]),'mean_query_Hz':statistics.mean(val(38)),'dma_overflows':int(rr[-1][42])}
    Path(str(stem)+'.summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
    print(json.dumps(summary,indent=2),flush=True)
    if failure: raise SystemExit(1)
