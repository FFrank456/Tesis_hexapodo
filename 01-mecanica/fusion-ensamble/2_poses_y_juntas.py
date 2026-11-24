import adsk.core, adsk.fusion, math
def run(_context):
    app=adsk.core.Application.get()
    des=adsk.fusion.Design.cast(app.activeProduct); r=des.rootComponent
    tl0=des.timeline.count
    P={}
    for o in r.occurrences:
        if o.component.name.startswith('Pata '): P[int(o.component.name[5:])]={c.component.name:c for c in o.childOccurrences}; P[int(o.component.name[5:])]['o']=o
    ch=next(x for x in r.occurrences if x.component.name.startswith('Chasis'))
    ch.isGrounded=True
    # poses esperadas
    for k in range(2,7):
        for n in ('base_J0','coxa_link','femur_link','tibia_link'):
            m=P[1][n].transform2; m.transformBy(P[k]['o'].transform2); P[k][n].transform2=m
    if des.snapshots.hasPendingSnapshot: des.snapshots.add()
    exp={(k,n):P[k][n].transform2.asArray() for k in P for n in ('base_J0','coxa_link','femur_link','tibia_link')}
    for k in P: print(k,[round(v,3) for v in P[k]['base_J0'].transform2.asArray()[:8]],P[k]['base_J0'].isGrounded)
    def cyl(o,body,rad):
        b=next(x for x in o.bRepBodies if x.name==body)
        return next(f for f in b.faces if f.geometry.objectType.endswith('Cylinder') and abs(f.geometry.radius-rad)<1e-4)
    Ax=adsk.fusion.JointDirections.ZAxisJointDirection
    def rev(name,a,b,face):
      try:
          g=adsk.fusion.JointGeometry.createByNonPlanarFace(face,adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
          i=r.asBuiltJoints.createInput(a,b,g); i.setAsRevoluteJointMotion(Ax)
          j=r.asBuiltJoints.add(i); j.name=name
      except Exception as e: print('NO',name,str(e)[:90])
    for k in sorted(P):
        q=P[k]
        i=r.asBuiltJoints.createInput(q['base_J0'],ch,None); i.setAsRigidJointMotion()
        j=r.asBuiltJoints.add(i); j.name='fijo_base_J0_%d'%k
        rev('join_coxa_%d_J0'%k if k>1 else 'join_coxa_1_J0_v2',q['coxa_link'],q['base_J0'],cyl(q['coxa_link'],'J0_Eje',.15))
        rev('join_femur_%d_J1'%k,q['coxa_link'],q['femur_link'],cyl(q['coxa_link'],'Eje',.15))
        rev('join_tibia_%d_J2'%k,q['femur_link'],q['tibia_link'],cyl(q['femur_link'],'Eje',.15))
    for kk,v in exp.items():
        d=max(abs(a-b) for a,b in zip(v,P[kk[0]][kk[1]].transform2.asArray()))
        m=P[1][kk[1]].transform2; m.transformBy(P[kk[0]]['o'].transform2); e=max(abs(a-b) for a,b in zip(m.asArray(),P[kk[0]][kk[1]].transform2.asArray()))
        if d>1e-5 or e>1e-5: print('DESVIO',kk,d,e)
    for k in (): print(k,{n:[round(v,3) for v in (P[k][n].transform2.translation.x,P[k][n].transform2.translation.y,P[k][n].transform2.translation.z)] for n in P[k]})
    for i in range(tl0,des.timeline.count):
        t=des.timeline.item(i); print(i,t.name,t.healthState)
