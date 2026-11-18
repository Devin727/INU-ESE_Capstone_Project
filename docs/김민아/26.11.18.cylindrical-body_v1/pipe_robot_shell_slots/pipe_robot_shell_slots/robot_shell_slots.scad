// Two cylindrical shells with six EMPTY wheel-module windows.
// Concept prototype: wheel/guide rods/LM4UU/springs/motors are NOT modeled.
// Units mm. Set part = "front", "rear", "assembly", or "joint".
$fn=72;
part="assembly";
shell_d=42; wall=2.5;
front_len=75; rear_len=105;
rear_x=167;
// Wheel-module opening in each shell, one per 120 degree direction.
// Opening extends axially along the robot and tangentially around its circumference.
slot_axial=38; slot_tangent=24;
front_slot_x=9; rear_slot_x=rear_x-5;
// shell rings stay intact near ends. Edges should be filleted after fit checks.
module shell_body(L,xc,slot_center){
  color("silver") difference(){
    translate([xc,0,0]) rotate([0,90,0])
      difference(){cylinder(h=L,d=shell_d,center=true);
                   cylinder(h=L+0.4,d=shell_d-2*wall,center=true);}
    // Three radial through-windows; nothing installed in the openings.
    for(a=[0:120:240])
      translate([xc,0,0]) rotate([a,0,0])
        translate([slot_center-xc,0,shell_d/2])
          cube([slot_axial,slot_tangent,20],center=true);
  }
}
module front_shell(){shell_body(front_len,0,front_slot_x);}
module rear_shell(){shell_body(rear_len,rear_x,rear_slot_x);}
// Front camera/LED plate is a separate printable part, not fused to shell.
module camera_plate(){
  color("#c5cdd5") translate([-front_len/2-3,0,0]) rotate([0,90,0])
    difference(){cylinder(h=4,d=42,center=true);
      cylinder(h=5,d=12,center=true);
      for(a=[0:60:300])translate([14*cos(a),14*sin(a),0])cylinder(h=5,d=3.2,center=true);
    }
}
// A visual placeholder only: existing multi-joint design is NOT print-ready.
module hinge_pin(x, vertical=true){
  color("#343f4a") translate([x,0,0])
    if(vertical) cylinder(h=24,d=4,center=true);
    else rotate([90,0,0])cylinder(h=24,d=4,center=true);
}
module hinge_fork(x,vertical=true){
  color("#8b9eaf") translate([x,0,0])
    if(vertical) for(z=[-9,9])translate([0,0,z])cube([12,12,5],center=true);
    else for(y=[-9,9])translate([0,y,0])cube([12,5,12],center=true);
}
module multi_link(x0,x1,vertical=true){
  color("#b7c1c9") translate([(x0+x1)/2,0,0])cube([x1-x0-8,11,11],center=true);
  hinge_fork(x0,vertical);hinge_fork(x1,!vertical);
}
module joint(){
  pts=[43,59,75,91,107];
  color("#71808d") for(x=[38,112])translate([x,0,0])rotate([0,90,0])cylinder(h=7,d=22,center=true);
  for(i=[0:3])multi_link(pts[i],pts[i+1],i%2==0);
  for(i=[0:4])hinge_pin(pts[i],i%2==0);
}
if(part=="front")front_shell();
else if(part=="rear")rear_shell();
else if(part=="camera_plate")camera_plate();
else if(part=="joint")joint();
else {front_shell();rear_shell();camera_plate();joint();}
