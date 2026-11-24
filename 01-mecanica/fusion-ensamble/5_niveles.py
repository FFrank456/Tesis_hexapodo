import adsk.core, adsk.fusion
def run(_c):
    app=adsk.core.Application.get(); d=adsk.fusion.Design.cast(app.activeProduct); r=d.rootComponent
    el=next(o for o in r.occurrences if o.component.name.startswith('Elec'))
    kids=list(el.childOccurrences)
    z={}
    key=lambda c:(c.component.name,tuple(round(v,4) for v in c.transform2.asArray()))
    before=sorted(key(c) for c in kids)
    lv={}
    for k in(1,2,3):
        q=el.component.occurrences.addNewComponent(adsk.core.Matrix3D.create()); q.component.name='Nivel %d'%k
        lv[k]=q.createForAssemblyContext(el)
    for c in kids:
        h=round(c.transform2.translation.z*10,1)
        k={17.0:1,32.8:2,48.6:3}.get(h)
        if k and c.component.name.startswith(('MOD_DRV','Marco')): c.moveToComponent(lv[k])
    after=[]
    for c in el.childOccurrences:
        if c.component.name.startswith('Nivel'):
            for x in c.childOccurrences: after.append(key(x)); print(c.component.name,x.name,round(x.transform2.translation.z*10,2))
        else: after.append(key(c)); print(c.name)
    if sorted(after)!=before: raise Exception('cambio posiciones')
    next(c for c in el.childOccurrences if c.component.name=='Bandeja_base').component.name='Tapa_PETG'
    print('OK',d.timeline.count)