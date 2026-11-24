import adsk.core, adsk.fusion, math
def run(_context):
    app=adsk.core.Application.get()
    des=adsk.fusion.Design.cast(app.activeProduct); r=des.rootComponent
    tl0=des.timeline.count
    get=lambda n: next(x for x in r.occurrences if x.component.name==n)
    N=('base_J0','coxa_link_1','femur_link_1','tibia_link_1')
    sig=lambda c: sorted((b.name,round(b.volume,4),b.faces.count,tuple(round(v,4) for v in b.boundingBox.minPoint.asArray())) for b in c.bRepBodies)
    before={n:sig(get(n)) for n in N}; chb=sig(get('Chasis (1)')); rb=sig(r)
    p=r.occurrences.addNewComponent(adsk.core.Matrix3D.create()); p.component.name='Pata 1'
    for n in N: get(n).moveToComponent(p)
    for c in p.childOccurrences:
        n=c.component.name
        if sig(c)!=before[n]: raise Exception('cambio '+n+str([x for x in zip(before[n],sig(c)) if x[0]!=x[1]][:3]))
    assert sig(get('Chasis (1)'))==chb,'chasis'
    assert sig(r)==rb,'raiz'
    Z=adsk.core.Vector3D.create(0,0,1); O=adsk.core.Point3D.create(0,0,0)
    H1=(8.127,5.07)
    legs=((2,3.1,4.109,-45),(3,-.927,5.07,-90),(4,-.927,11.135,180),(5,3.1,12.091,135),(6,8.127,11.135,90))
    local=[(c.component,c.transform2) for c in p.childOccurrences]
    for k,x,y,d in legs:
        M=adsk.core.Matrix3D.create(); M.setToRotation(math.radians(d),Z,O)
        v=adsk.core.Vector3D.create(H1[0],H1[1],0); v.transformBy(M)
        M.translation=adsk.core.Vector3D.create(x-v.x,y-v.y,0)
        q=r.occurrences.addNewComponent(M); q.component.name='Pata %d'%k
        for comp,t in local: q.component.occurrences.addExistingComponent(comp,t)
    for c,n in zip([c.component for c in p.childOccurrences],('base_J0','coxa_link','femur_link','tibia_link')): c.name=n
    for o in r.occurrences:
        print(o.name)
        if o.component.name.startswith('Pata'):
            for c in o.childOccurrences:
                b=c.boundingBox; print('   ',c.name,c.bRepBodies.count,c.isGrounded,[round(10*v,1) for v in b.minPoint.asArray()+b.maxPoint.asArray()])
    for i in range(tl0,des.timeline.count):
        t=des.timeline.item(i)
        if t.healthState: print('!!',t.name,t.healthState)
    print(tl0,des.timeline.count)
