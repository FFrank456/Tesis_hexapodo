import adsk.core, adsk.fusion
def run(_c):
    app=adsk.core.Application.get(); d=adsk.fusion.Design.cast(app.activeProduct); r=d.rootComponent
    el=next(o for o in r.occurrences if o.component.name.startswith('Elec'))
    if d.snapshots.hasPendingSnapshot: raise Exception('hay posiciones pendientes del usuario')
    pat={o.fullPathName:o.transform2.asArray() for p in r.occurrences if p.component.name.startswith('Pata') for o in p.childOccurrences}
    for c in el.childOccurrences:
        dz=-.81 if c.component.name=='Bandeja_base' else .69
        m=c.transform2; t=m.translation; t.z+=dz; m.translation=t; c.transform2=m
    if d.snapshots.hasPendingSnapshot: d.snapshots.add()
    for c in el.childOccurrences: print(c.name,round(c.transform2.translation.z*10,2),[ (b.name,round(b.boundingBox.minPoint.z*10,2),round(b.boundingBox.maxPoint.z*10,2)) for b in c.bRepBodies][:2])
    for p in r.occurrences:
        if p.component.name.startswith('Pata'):
            for o in p.childOccurrences:
                e=max(abs(a-b) for a,b in zip(pat[o.fullPathName],o.transform2.asArray()))
                if e>1e-6: raise Exception('se movio '+o.fullPathName)
    ch=next(o for o in r.occurrences if o.component.name.startswith('Chasis')).bRepBodies.item(0)
    tapa=next(c for c in el.childOccurrences if c.component.name=='Bandeja_base').bRepBodies.item(0)
    T=adsk.fusion.TemporaryBRepManager.get()
    def inter(a,b):
        x=T.copy(a); y=T.copy(b)
        try:
            T.booleanOperation(x,y,adsk.fusion.BooleanTypes.IntersectionBooleanType)
            v=x.volume*1000; bb=x.boundingBox
            return round(v,2),[round(q*10,1) for q in bb.minPoint.asArray()+bb.maxPoint.asArray()] if v>0 else None
        except Exception as ex: return 'err '+str(ex)[:60]
    print('tapa-chasis',inter(tapa,ch))
    for p in r.occurrences:
        if p.component.name.startswith('Pata'):
            for c in p.childOccurrences:
                if c.component.name=='base_J0':
                    for b in c.bRepBodies:
                        if b.boundingBox.maxPoint.z>1.12 and b.boundingBox.intersects(tapa.boundingBox):
                            q=inter(b,tapa)
                            if isinstance(q,str) or q[0]>.01: print(p.name,b.name,q)
    sep=next(c for c in el.childOccurrences if c.component.name.startswith('Sepa'))
    for b in sep.bRepBodies:
        if b.name[-2:] in('07','08','09','10','11','12'): print('sep',b.name,inter(b,tapa))
    n=0
    for c in el.childOccurrences:
        if c.name in('MOD_DRV:1','MOD_DRV:2'):
            stack=[c]
            while stack:
                o=stack.pop(); stack+=list(o.childOccurrences)
                for b in o.bRepBodies:
                    if b.isSolid and b.boundingBox.minPoint.z<2.02 and b.boundingBox.intersects(tapa.boundingBox):
                        q=inter(b,tapa); n+=1
                        if isinstance(q,str) or q[0]>.05: print(c.name,o.name,b.name,q)
                    if b.isSolid and b.boundingBox.minPoint.z<1.33 and b.boundingBox.intersects(ch.boundingBox):
                        q=inter(b,ch)
                        if isinstance(q,str) or q[0]>.05: print('CH',c.name,o.name,b.name,q)
    print('probados',n)