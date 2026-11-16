import copy, json, math
from pathlib import Path
import xml.etree.ElementTree as E
root=Path(__file__).parent
out=root/'comparison'; out.mkdir(exist_ok=True)
base=E.parse(root/'co2_pipe.sdf')
cases=[('baseline_hinge_3N',False,3),('case1_rigid_3N',True,3),('case2_hinge_6N',False,6),('case3_rigid_6N',True,6)]
for name,rigid,force in cases:
 t=copy.deepcopy(base); model=t.find("./world/model[@name='robot']"); plugin=model.find('plugin'); plugin.find('thrust').text=str(force)
 E.SubElement(plugin,'telemetry').text=f'comparison/{name}.csv'
 if rigid:
  rear=model.find("link[@name='rear']"); front=model.find("link[@name='front']")
  model.remove(rear); model.remove(front); model.remove(model.find("joint[@name='hinge']"))
  body=copy.deepcopy(rear); body.set('name','chassis'); body.find('pose').text='0 0 0 0 0 0'; body.find('inertial/mass').text='0.21'
  dims=(.12,.025,.025)
  for k,v in zip(('ixx','iyy','izz'),(.21*(dims[1]**2+dims[2]**2)/12,.21*(dims[0]**2+dims[2]**2)/12,.21*(dims[0]**2+dims[1]**2)/12)):
   body.find('inertial/inertia/'+k).text=str(v)
  body.find('collision/geometry/box/size').text='0.12 0.025 0.025'; body.find('visual/geometry/box/size').text='0.12 0.025 0.025'
  model.insert(1,body)
  for j in model.findall('joint'):
   if j.findtext('parent') in ('front','rear'):j.find('parent').text='chassis'
  E.SubElement(plugin,'body_link').text='chassis'
 mass=sum(float(x.text) for x in model.findall('link/inertial/mass'))
 assert abs(mass-.3)<1e-8,(name,mass)
 t.write(out/(name+'.sdf'),encoding='utf-8',xml_declaration=True)
 print(name,'mass=',round(mass,4),'kg')
