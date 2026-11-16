"""20 g payload increments; stop on first verified timeout, record both entry and traversal."""
import copy,csv,json,os,signal,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parent;os.chdir(ROOT)
OUT=ROOT/'mass_sweep';OUT.mkdir(exist_ok=True)
BASE=E.parse(ROOT/'hvh/rigid_3N_hvh.sdf')
results=json.loads((OUT/'results.json').read_text()) if (OUT/'results.json').exists() else []
def save():
 (OUT/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
 cols=['mass_g','seed','limit_s','success','vertical_time','upper_entry_time','upper_end_time','last_sim_time','stop_reason','file']
 with (OUT/'results.csv').open('w',newline='',encoding='utf-8-sig') as f:
  writer=csv.DictWriter(f,fieldnames=cols,extrasaction='ignore');writer.writeheader();writer.writerows(results)
def run(mass,limit=30,seed=42,label='sweep'):
 name=f'{mass}g_{label}_seed{seed}_{limit}s';tree=copy.deepcopy(BASE)
 model=tree.find("./world/model[@name='robot']");body=model.find("link[@name='chassis']")
 new_mass=mass/1000-.09;ratio=new_mass/float(body.findtext('inertial/mass'))
 body.find('inertial/mass').text=str(new_mass)
 for e in body.findall('inertial/inertia/*'):e.text=str(float(e.text)*ratio)
 assert abs(sum(float(e.text) for e in model.findall('link/inertial/mass'))-mass/1000)<1e-8
 plugin=model.find('plugin');assert plugin.findtext('thrust')=='3' and plugin.findtext('continuous')=='true'
 plugin.find('telemetry').text=f'mass_sweep/{name}.csv'
 sdf=OUT/(name+'.sdf');tree.write(sdf,encoding='utf-8',xml_declaration=True)
 env=os.environ.copy();env['GZ_PARTITION']='mass_'+name;env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(ROOT/'build')
 log=(OUT/(name+'.log')).open('w');p=subprocess.Popen(['gz','sim','-s','-r','--iterations',str(int(limit*1000)),'--seed',str(seed),'-v','2',str(sdf)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 print('START',mass,'g',limit,'sim seconds','seed',seed,flush=True)
 started=time.monotonic();stop='completed';rows=[];goal=None
 def read_rows():
  path=OUT/(name+'.csv')
  if not path.exists():return []
  try:
   with path.open() as f:return [r for r in csv.DictReader(f) if r.get('model_z') is not None]
  except (OSError,csv.Error):return []
 def first(rows,predicate):
  for row in rows:
   try:r={k:float(v) for k,v in row.items()}
   except (ValueError,TypeError):continue
   if predicate(r):return r['time']
  return None
 def vertical(r):return abs(r['x']-.75)<.04 and abs(r['y'])<.04 and .5<=r['z']<=1.05 and r['forward_z']>.7
 def upper(r):return 1.0<=r['x']<=1.5 and abs(r['y'])<.04 and abs(r['z']-1.2)<.04 and abs(r['forward_z'])<.3
 try:
  while p.poll() is None:
   time.sleep(1)
   rows=read_rows();goal=first(rows,lambda r:upper(r) and r['x']>=1.4)
   if goal is not None:stop='goal';break
   if time.monotonic()-started>600:stop='wall_timeout';break
 finally:
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  log.close()
 rows=read_rows()
 if not rows:raise RuntimeError('No telemetry: '+name)
 goal=first(rows,lambda r:upper(r) and r['x']>=1.4)
 result={'mass_g':mass,'seed':seed,'limit_s':limit,'success':goal is not None,
 'vertical_time':first(rows,vertical),'upper_entry_time':first(rows,upper),'upper_end_time':goal,
 'last_sim_time':float(rows[-1]['time']),'stop_reason':stop,'file':name,
 'last':rows[-1],'max_z':max(float(r['z']) for r in rows)}
 if stop=='wall_timeout':result['inconclusive']=True
 results.append(result);save();print(json.dumps(result),flush=True);return result
last_ok=max((r['mass_g'] for r in results if r['success']),default=None)
for mass in range((last_ok+20) if last_ok else 300,2001,20):
 result=run(mass)
 if result.get('inconclusive'):raise RuntimeError('Wall time limit; not a physical failure')
 if result['success']:last_ok=mass;continue
 extended=run(mass,60,42,'extended')
 if extended.get('inconclusive'):raise RuntimeError('Extended test incomplete')
 if extended['success']:last_ok=mass;continue
 if last_ok is not None:run(last_ok,60,43,'repeat')
 run(mass,60,43,'repeat')
 summary={'last_success_g':last_ok,'first_failure_g':mass,'step_g':20,'criterion':'front wheel station reaches upper horizontal x>=1.4m within 60 simulated seconds','mass_added_to':'chassis, with inertia scaled; wheels and carriers unchanged'}
 (OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8');print('SUMMARY',json.dumps(summary),flush=True);break
else:print('No failure found up to 2000g',flush=True)
