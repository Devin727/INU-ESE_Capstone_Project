"""Extend the original rigid 3N case without replacing the comparison files."""
import importlib.util
import xml.etree.ElementTree as E
from pathlib import Path
import math
root=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('pipe_geometry',root/'generate.py')
g=importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
old=len(g.segments)
R=.15
# Existing vertical terminates at x=.75, z=1.05. Turn toward +X.
for i in range(12):
 a=(i+.5)*(math.pi/2)/12
 g.segment(.90-R*math.cos(a),1.05+R*math.sin(a),math.pi/2-a,R*math.pi/24+.003,'upper_bend_'+str(i))
for i in range(6):g.segment(.90+(i+.5)*.1,1.20,0,.103,'upper_horizontal_'+str(i))
tree=E.parse(root/'comparison'/'case1_rigid_3N.sdf'); world=tree.find('world');world.set('name','co2_hvh')
pipe=world.find("model[@name='pipe']")
for fragment in g.segments[old:]:pipe.append(E.fromstring(fragment))
# All pipe pieces are static; aggregate into one rigid link without changing geometry.
merged=E.Element('link',{'name':'pipe_shell'})
for link in list(pipe.findall('link')):
 pose=link.findtext('pose','0 0 0 0 0 0')
 for item in list(link):
  if item.tag in ('collision','visual'):
   item.set('name',link.get('name')+'_'+item.get('name'))
   E.SubElement(item,'pose').text=pose
   merged.append(item)
 pipe.remove(link)
pipe.append(merged)
plugin=world.find("model[@name='robot']/plugin")
plugin.find('telemetry').text='hvh/trajectory.csv'
assert plugin.findtext('thrust')=='3'
assert plugin.findtext('continuous')=='true'
assert plugin.findtext('trigger_x')=='0.40'
assert abs(sum(float(e.text) for e in world.findall("model[@name='robot']/link/inertial/mass"))-.3)<1e-8
out=root/'hvh';out.mkdir(exist_ok=True)
tree.write(out/'rigid_3N_hvh.sdf',encoding='utf-8',xml_declaration=True)
print('Created horizontal 0.60m -> elbow -> vertical 0.60m -> elbow -> horizontal 0.60m; 300g,3N continuous')
