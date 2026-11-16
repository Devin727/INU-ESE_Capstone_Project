import math
from pathlib import Path
pi=math.pi
# Approximation: nominal diameter and elbow curvature are placeholders, not measured dimensions.
def euler(m):
    pitch=math.asin(max(-1,min(1,-m[2][0])))
    if abs(math.cos(pitch))>1e-7: return math.atan2(m[2][1],m[2][2]),pitch,math.atan2(m[1][0],m[0][0])
    return 0,pitch,math.atan2(-m[0][1],m[1][1])
def frame(theta,phi):
    t=(math.cos(theta),0,math.sin(theta)); n=(-math.sin(theta),0,math.cos(theta)); b=(0,1,0)
    r=tuple(math.cos(phi)*b[i]+math.sin(phi)*n[i] for i in range(3))
    q=tuple(-math.sin(phi)*b[i]+math.cos(phi)*n[i] for i in range(3))
    # r x q = t
    m=[[r[i],q[i],t[i]] for i in range(3)]
    return r,euler(m)
def vec(v): return ' '.join(f'{x:.8f}' for x in v)
segments=[]
def segment(x,z,theta,length,k):
    for j in range(16):
        r,angles=frame(theta,2*pi*j/16); pos=(x+0.052*r[0],0.052*r[1],z+0.052*r[2]); pose=vec(pos)+' '+vec(angles)
        geom=f'<box><size>0.004 0.0205 {length:.6f}</size></box>'
        segments.append(f'<link name="tube_{k}_{j}"><pose>{pose}</pose><collision name="c"><geometry>{geom}</geometry><surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface></collision><visual name="v"><transparency>0.80</transparency><geometry>{geom}</geometry><material><ambient>0.1 0.55 0.85 1</ambient><diffuse>0.1 0.55 0.85 1</diffuse></material></visual></link>')
for i in range(6): segment((i+0.5)*0.1,0.3,0,0.103,'h'+str(i))
R=.15
for i in range(12):
    t=(i+.5)*(pi/2)/12; segment(.6+R*math.sin(t),.45-R*math.cos(t),t,R*(pi/2)/12+0.003,'b'+str(i))
for i in range(6): segment(.75,.45+(i+.5)*.1,pi/2,.103,'v'+str(i))
def inertia(m,x,y,z): return f'<inertial><mass>{m}</mass><inertia><ixx>{m*(y*y+z*z)/12:.9f}</ixx><iyy>{m*(x*x+z*z)/12:.9f}</iyy><izz>{m*(x*x+y*y)/12:.9f}</izz></inertia></inertial>'
links=[]; joints=[]
for name,x in [('rear',-.0325),('front',.0325)]:
    geom='<box><size>0.055 0.025 0.025</size></box>'
    links.append(f'<link name="{name}"><pose>{x} 0 0 0 0 0</pose>{inertia(.105,.055,.025,.025)}<collision name="c"><geometry>{geom}</geometry></collision><visual name="v"><geometry>{geom}</geometry><material><ambient>0.8 0.24 0.08 1</ambient><diffuse>0.8 0.24 0.08 1</diffuse></material></visual></link>')
joints.append('<joint name="hinge" type="revolute"><parent>rear</parent><child>front</child><pose relative_to="__model__">0 0 0 0 0 0</pose><axis><xyz>0 1 0</xyz><limit><lower>-1.2</lower><upper>1.2</upper></limit><dynamics><damping>0.004</damping></dynamics></axis></joint>')
for i in range(6):
    mod='rear' if i<3 else 'front'; x=-.0325 if i<3 else .0325; a=2*pi*(i%3)/3; y=.034*math.cos(a); z=.034*math.sin(a); radial=f'0 {math.cos(a)} {math.sin(a)}'; tangent=f'0 {-math.sin(a)} {math.cos(a)}'
    links.append(f'<link name="carrier_{i}"><pose>{x} {y} {z} 0 0 0</pose>{inertia(.003,.006,.006,.006)}</link>')
    wheel='<cylinder><radius>0.015</radius><length>0.006</length></cylinder>'
    links.append(f'<link name="wheel_{i}"><pose>{x} {y} {z} {a} 0 0</pose>{inertia(.012,.03,.03,.006)}<collision name="c"><geometry>{wheel}</geometry><surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision><visual name="v"><geometry>{wheel}</geometry><material><ambient>0.06 0.06 0.06 1</ambient><diffuse>0.06 0.06 0.06 1</diffuse></material></visual></link>')
    joints.append(f'<joint name="spring_{i}" type="prismatic"><parent>{mod}</parent><child>carrier_{i}</child><axis><xyz expressed_in="__model__">{radial}</xyz><limit><lower>-0.007</lower><upper>0.004</upper></limit><dynamics><spring_reference>0.004</spring_reference><spring_stiffness>100</spring_stiffness><damping>0.3</damping></dynamics></axis></joint>')
    joints.append(f'<joint name="wheel_joint_{i}" type="revolute"><parent>carrier_{i}</parent><child>wheel_{i}</child><axis><xyz expressed_in="__model__">{tangent}</xyz><dynamics><damping>0.0001</damping></dynamics></axis></joint>')
s=f'''<?xml version="1.0"?><sdf version="1.9"><world name="co2_pipe"><physics name="p" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.81</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><light name="sun" type="directional"><pose>0 0 3 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 -0.5 -1</direction></light><model name="pipe"><static>true</static>{''.join(segments)}</model><model name="robot"><pose>0.12 0 0.3 0 0 0</pose>{''.join(links)}{''.join(joints)}<plugin filename="libPipeControl.so" name="PipeControl"><thrust>3.0</thrust><duration>2.0</duration><continuous>true</continuous><trigger_x>0.40</trigger_x></plugin></model></world></sdf>'''
Path(__file__).with_name('co2_pipe.sdf').write_text(s,encoding='utf-8')
