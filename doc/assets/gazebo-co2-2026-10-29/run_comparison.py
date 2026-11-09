import subprocess, os, time, signal, csv, json
from pathlib import Path
root=Path(__file__).resolve().parent; os.chdir(root)
cases=['baseline_hinge_3N','case1_rigid_3N','case2_hinge_6N','case3_rigid_6N']; results=[]
for name in cases:
 env=os.environ.copy();env['GZ_PARTITION']='comparison_'+name;env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(root/'build')
 valid=subprocess.run(['gz','sdf','-k',str(root/'comparison'/(name+'.sdf'))],capture_output=True,text=True)
 if valid.returncode:raise RuntimeError(valid.stdout+valid.stderr)
 print('START',name,flush=True)
 with open(root/'comparison'/(name+'.log'),'w') as log:
  p=subprocess.Popen(['gz','sim','-s','-r','--iterations','12000','--seed','42','-v','2',str(root/'comparison'/(name+'.sdf'))],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  try: code=p.wait(timeout=360)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM);code=-1
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
 rows=list(csv.DictReader(open(root/'comparison'/(name+'.csv'))))
 if not rows:raise RuntimeError('No telemetry: '+name)
 on=[r for r in rows if r['gas_on']=='1']
 # Front wheel station enters straight vertical pipe at x~0.75,z>=0.45.
 inside=[r for r in rows if abs(float(r['x'])-.75)<.04 and abs(float(r['y']))<.04 and float(r['z'])>=.50 and float(r['forward_z'])>.7]
 result={'case':name,'exit':code,'sim_end':float(rows[-1]['time']),'gas_first_sample':float(on[0]['time']) if on else None,'max_z':max(float(r['z']) for r in rows),'top_time':next((float(r['time']) for r in rows if abs(float(r['x'])-.75)<.04 and abs(float(r['y']))<.04 and float(r['z'])>=.95 and float(r['forward_z'])>.7),None),'vertical_entered':bool(inside),'end':{k:float(rows[-1][k]) for k in ['x','y','z','forward_z']}}
 results.append(result);(root/'comparison'/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
