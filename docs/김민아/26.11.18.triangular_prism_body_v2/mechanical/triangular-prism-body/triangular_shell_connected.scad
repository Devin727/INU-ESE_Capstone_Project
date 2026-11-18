// Triangular PRISM CLOSED shells with EMPTY wheel-module openings.
// Conceptual geometry, units mm. Wheel/guide rod/LM4UU/spring/motor not included.
$fn=64;
part="assembly"; // front, rear, camera_plate, joint, assembly
outer_R=24; wall=2.5; corner_radius=2;
front_len=75; rear_len=105; rear_x=167;
slot_axial=38; slot_tangent=24;
front_slot_x=9; rear_slot_x=rear_x-5;
// Cross section in yz, vertices point upwards and down diagonally.
// 2D x is -z after rotate([0,90,0]); 2D y is y.
module tri2d(R){
  polygon(points=[for(a=[90,210,330]) [R*cos(a),R*sin(a)]]);
}
module profile_outer(){offset(r=corner_radius) offset(delta=-corner_radius) tri2d(outer_R);}
module profile_inner(){offset(delta=-wall) profile_outer();}
module shell_base(L,xc){
 translate([xc,0,0])rotate([0,90,0])
 difference(){
   linear_extrude(height=L,center=true) profile_outer();
   linear_extrude(height=L+0.5,center=true) profile_inner();
 }
}
// Correct face normals are 90,210,330 degrees after profile rotation.
// Slots cut face centers only, preserving continuous triangular corner rails.
// Opening is cut through wall on each of the three FLAT faces.
module empty_slots(xc,slot_center){
 for(a=[90,210,330])
   rotate([a,0,0]) translate([slot_center,0,outer_R/2])
      cube([slot_axial,slot_tangent,9],center=true);
}
module shell(L,xc,slot_center){
 difference(){shell_base(L,xc);empty_slots(xc,slot_center);}
}
module front_shell(){color("#3b82f6")shell(front_len,0,front_slot_x);}
module rear_shell(){color("#f59e0b")shell(rear_len,rear_x,rear_slot_x);}
module camera_plate(){
 color("#94a3b8")translate([-front_len/2-3,0,0])rotate([0,90,0])
 difference(){linear_extrude(height=4,center=true)profile_outer();
 cylinder(h=5,d=12,center=true);
 for(a=[0:120:240])translate([12*cos(a),12*sin(a),0])cylinder(h=5,d=3.2,center=true);
 }
}
// Conceptual joint only; not a mechanically verified printable articulation.
module hinge_pin(x,vertical=true){
 color("#343f4a")translate([x,0,0])
 if(vertical)cylinder(h=24,d=4,center=true);
 else rotate([90,0,0])cylinder(h=24,d=4,center=true);
}
module hinge_fork(x,vertical=true){
 color("#8b9eaf")translate([x,0,0])
 if(vertical)for(z=[-9,9])translate([0,0,z])cube([12,12,5],center=true);
 else for(y=[-9,9])translate([0,y,0])cube([12,5,12],center=true);
}
module multi_link(x0,x1,vertical=true){
 color("#b7c1c9")translate([(x0+x1)/2,0,0])cube([x1-x0-8,11,11],center=true);
 hinge_fork(x0,vertical);hinge_fork(x1,!vertical);
}
module joint(){
 pts=[43,59,75,91,107];
 color("#71808d")for(x=[38,112])translate([x,0,0])rotate([0,90,0])cylinder(h=7,d=22,center=true);
 for(i=[0:3])multi_link(pts[i],pts[i+1],i%2==0);
 for(i=[0:4])hinge_pin(pts[i],i%2==0);
}
if(part=="front")front_shell();
else if(part=="rear")rear_shell();
else if(part=="camera_plate")camera_plate();
else if(part=="joint")joint();
else{front_shell();rear_shell();camera_plate();joint();}
