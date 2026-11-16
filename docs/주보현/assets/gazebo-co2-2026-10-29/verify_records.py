"""Recompute recorded Gazebo classifications; no new physics simulations are run."""
from pathlib import Path
import csv,json,math,hashlib,xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parent
checks=0

def check(condition,message):
 global checks
 if not condition:raise AssertionError(message)
 checks+=1

def rows(path):
 with path.open(encoding='utf-8-sig') as f:return [{k:float(v) for k,v in row.items()} for row in csv.DictReader(f)]

def first(data,pred):return next((r['time'] for r in data if pred(r)),None)
def close(a,b):return a is None and b is None or a is not None and b is not None and abs(a-b)<1e-6

def upper(r):return 1.0<=r['x']<=1.5 and abs(r['y'])<.04 and abs(r['z']-1.2)<.04 and abs(r['forward_z'])<.3

def model_checks(path,mass,force):
 tree=E.parse(path);w=tree.find('world');m=w.find("model[@name='robot']")
 check(w.findtext('gravity')=='0 0 -9.81',str(path)+' gravity')
 check(abs(sum(float(x.text) for x in m.findall('link/inertial/mass'))-mass)<1e-8,str(path)+' mass')
 check(float(m.findtext('plugin/thrust'))==force,str(path)+' thrust')
 check(m.findtext('plugin/continuous')=='true',str(path)+' continuous')
 check(float(m.findtext('plugin/trigger_x'))==.4,str(path)+' trigger')
 check(len([x for x in m.findall('link') if x.get('name','').startswith('wheel_')])==6,str(path)+' wheels')
 return w,m

mass_results=json.loads((ROOT/'mass_sweep/results.json').read_text())
for res in mass_results:
 name=res['file'];data=rows(ROOT/'mass_sweep'/f'{name}.csv');w,m=model_checks(ROOT/'mass_sweep'/f'{name}.sdf',res['mass_g']/1000,3)
 check(m.find("joint[@name='hinge']") is None,name+' no hinge')
 check(close(first(data,upper),res['upper_entry_time']),name+' upper entry')
 goal=first(data,lambda r:upper(r) and r['x']>=1.4)
 check(close(goal,res['upper_end_time']),name+' goal time');check((goal is not None)==res['success'],name+' outcome')
 check(close(data[-1]['time'],res['last_sim_time']),name+' end time')
 check(close(max(r['z'] for r in data),res['max_z']),name+' max z')
 fired=next(i for i,r in enumerate(data) if r['gas_on']==1)
 check(all(r['gas_on']==1 for r in data[fired:]),name+' uninterrupted thrust')
 check('CO2 ON' in (ROOT/'mass_sweep'/f'{name}.log').read_text(),name+' trigger log')
 if not res['success']:check(res['last_sim_time']>=res['limit_s']-.06,name+' full failure observation')
summary=json.loads((ROOT/'mass_sweep/summary.json').read_text())
primary=[r for r in mass_results if '_sweep_' in r['file']]
check([r['mass_g'] for r in primary]==list(range(300,421,20)),'20g coverage')
check(max(r['mass_g'] for r in primary if r['success'])==summary['last_success_g']==400,'last success')
check(min(r['mass_g'] for r in primary if not r['success'])==summary['first_failure_g']==420,'first failure')
for res in json.loads((ROOT/'comparison/results.json').read_text()):
 name=res['case'];data=rows(ROOT/'comparison'/f'{name}.csv');model_checks(ROOT/'comparison'/f'{name}.sdf',.3,6 if '6N' in name else 3)
 predicate=lambda r:abs(r['x']-.75)<.04 and abs(r['y'])<.04 and r['z']>=.95 and r['forward_z']>.7
 check(close(first(data,predicate),res['top_time']),name+' recorded top time')
model_checks(ROOT/'hvh/rigid_3N_hvh.sdf',.3,3)
# Hash manifest makes subsequent changes to the source evidence detectable.
manifest={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes().replace(b'\r\n', b'\n')).hexdigest() for p in sorted(ROOT.rglob('*')) if p.is_file() and p.name not in ('verification.json','SHA256.json')}
(ROOT/'SHA256.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
output={'verified_on':'2026-11-09','checks_passed':checks,'mass_runs_checked':len(mass_results),'comparison_runs_checked':4,'hvh_runs_checked':0,'excluded_evidence':'hvh/trajectory.csv was overwritten during GUI replay and differs from hvh/results.json; use mass_sweep/300g_sweep_seed42_30s.csv for the final 300g path result','new_physics_runs':0,'scope':'Recorded results recalculated from CSV and checked against SDF/control source; not physical validation'}
(ROOT/'verification.json').write_text(json.dumps(output,indent=2),encoding='utf-8');print(json.dumps(output))
