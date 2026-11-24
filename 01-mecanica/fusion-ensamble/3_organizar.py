import adsk.core, adsk.fusion, math
def run(_context):
    app=adsk.core.Application.get()
    des=adsk.fusion.Design.cast(app.activeProduct); r=des.rootComponent
    for js in (r.joints,r.asBuiltJoints):
        for a,b in(('join_femur_1_j1','join_femur_1_J1'),('join_femur_1_j2','join_tibia_1_J2')):
            j=js.itemByName(a)
            if j: j.name=b
    n=r.bRepBodies.count
    o=r.occurrences.addNewComponent(adsk.core.Matrix3D.create()); o.component.name='Copia suelta pata 2 (original)'
    col=adsk.core.ObjectCollection.create()
    for b in r.bRepBodies: col.add(b)
    o.component.features.cutPasteBodies.add(col)
    print('cuerpos',n,'-> raiz',r.bRepBodies.count,'copia',o.component.bRepBodies.count)
    o.isLightBulbOn=False
    tl=des.timeline
    # indice del primer/ultimo elemento del ensamble
    idx=[i for i in range(tl.count) if tl.item(i).name.strip()=='Pata 1:1']
    print(idx,tl.count)
    for i in range(1024,tl.count):
        t=tl.item(i)
        if t.healthState: print('!!',t.name,t.healthState)
    try:
        g=tl.timelineGroups.add(idx[-1],tl.count-1); g.name='Ensamble hexapodo'; print('grupo ok')
    except Exception as e: print('grupo',e)
