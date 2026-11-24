"""f.py run script.py [ro] | read '{json}' [out.png] | upd undo"""
import sys, json, base64, urllib.request
sys.stdout.reconfigure(encoding='utf-8')
U='http://127.0.0.1:64904/mcp'; sid=None
def post(m):
    global sid
    h={'Content-Type':'application/json','Accept':'application/json, text/event-stream'}
    if sid: h['Mcp-Session-Id']=sid
    r=urllib.request.urlopen(urllib.request.Request(U,json.dumps(m).encode(),h),timeout=580)
    sid=r.headers.get('Mcp-Session-Id') or sid
    b=r.read().decode('utf-8','replace')
    return json.loads(b) if b.strip().startswith('{') else b
post({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-03-26","capabilities":{},"clientInfo":{"name":"cli","version":"1"}}})
post({"jsonrpc":"2.0","method":"notifications/initialized"})
k=sys.argv[1]
if k=='run':
    n='fusion_mcp_execute'; a={"featureType":"script","object":{"script":open(sys.argv[2],encoding='utf-8').read(),"readOnly":len(sys.argv)>3}}
elif k=='read': n='fusion_mcp_read'; a=json.loads(open(sys.argv[2]).read()) if sys.argv[2].endswith('.json') else dict(p.split('=') for p in sys.argv[2].split(','))
elif k=='upd': n='fusion_mcp_update'; a={"featureType":sys.argv[2]}
if k=='read':
    for q in ('width','height'):
        if q in a: a[q]=int(a[q])
r=post({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":n,"arguments":a}})
for c in r.get('result',{}).get('content',[]) if isinstance(r,dict) else []:
    d=c.get('data')
    if c.get('type')=='text':
        try:
            j=json.loads(c['text']); d=j.get('base64Data') or d
            if not d: print(j.get('message') or j.get('output') or c['text'] if isinstance(j,dict) else c['text'])
            if isinstance(j,dict) and not j.get('success',True): print(c['text'])
        except Exception: print(c['text'])
    if d: open(sys.argv[3],'wb').write(base64.b64decode(d)); print('saved')
if 'error' in r: print(r['error'])
