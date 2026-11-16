import os,subprocess,signal,csv,json
from pathlib import Path
root=Path(__file__).resolve().parent;os.chdir(root)
env=os.environ.copy();env['GZ_PARTITION']='hvh_validation';env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(root/'build')
r=subprocess.run(['gz','sdf','-k','hvh/rigid_3N_hvh.sdf'],capture_output=True,text=True)
print(r.stdout,r.stderr,flush=True)
if r.returncode:raise RuntimeError('SDF validation failed')
with open(root/'hvh/test.log','w') as log:
 p=subprocess.Popen(['gz','sim','-s','-r','--iterations','12000','--seed','42','-v','2','hvh/rigid_3N_hvh.sdf'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:code=p.wait(timeout=480)
 except subprocess.TimeoutExpired:
  os.killpg(p.pid,signal.SIGTERM);code=-1
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
rows=list(csv.DictReader(open(root/'hvh/trajectory.csv')))
def first(pred):return next((float(r['time']) for r in rows if pred({k:float(v) for k,v in r.items()})),None)
result={'exit_code':code,'sim_end':float(rows[-1]['time']),
 'vertical_entry_time':first(lambda r:abs(r['x']-.75)<.04 and abs(r['y'])<.04 and .5<=r['z']<=1.05 and r['forward_z']>.7),
 'upper_horizontal_entry_time':first(lambda r:1.0<=r['x']<=1.5 and abs(r['y'])<.04 and abs(r['z']-1.2)<.04 and abs(r['forward_z'])<.3),
 'upper_horizontal_end_time':first(lambda r:1.4<=r['x']<=1.5 and abs(r['y'])<.04 and abs(r['z']-1.2)<.04 and abs(r['forward_z'])<.3),
 'end':rows[-1]}
(root/'hvh/results.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
