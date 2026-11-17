// PVC pipe inspection robot: modular articulated connector (prototype)
// Units mm. OpenSCAD 2021+.
// Select: "assembly", "link", "front_mount", "rear_mount", "washer".
// IMPORTANT: planar 1-DOF hinges only; not a universal joint.
part = "assembly";
link_count = 4;
pitch = 26;
pin_d = 4.0;
hole_clearance = 0.25; // diameter clearance; tune to printer
hinge_r = 7.0;
tongue_thick = 4.6;
ear_thick = 3.2;
side_gap = 0.45; // clearance per side
spine_width = 12;
spine_thick = 10;
$fn = 64;

hole_d = pin_d + hole_clearance;
ear_center = tongue_thick/2 + side_gap + ear_thick/2;

module disk_at(x, z, t) {
    translate([x,0,z]) cylinder(r=hinge_r,h=t,center=true);
}
module bore_at(x) {
    translate([x,0,0]) cylinder(d=hole_d,h=32,center=true);
}
module link() {
    difference() {
        union() {
            // left: central tongue, right: two-sided clevis
            disk_at(-pitch/2, 0, tongue_thick);
            disk_at(pitch/2, ear_center, ear_thick);
            disk_at(pitch/2, -ear_center, ear_thick);
            cube([pitch-13,spine_width,spine_thick],center=true);
            // ribs linking clevis ears to central spine
            for (sgn=[-1,1])
                hull() {
                    translate([pitch/2-7,0,sgn*ear_center]) cylinder(r=4,h=ear_thick,center=true);
                    translate([pitch/2-9,0,sgn*spine_thick/2]) cylinder(r=4,h=2,center=true);
                }
        }
        bore_at(-pitch/2);
        bore_at(pitch/2);
    }
}
module front_mount() {
    difference() {
        union() {
            translate([-10,0,0]) cube([20,spine_width,spine_thick],center=true);
            for (sgn=[-1,1]) disk_at(0,sgn*ear_center,ear_thick);
        }
        bore_at(0);
        // mounting holes: M3 clearance through z
        for (x=[-16,-8]) translate([x,0,0]) cylinder(d=3.3,h=30,center=true);
    }
}
module rear_mount() {
    difference() {
        union() {
            translate([10,0,0]) cube([20,spine_width,spine_thick],center=true);
            disk_at(0,0,tongue_thick);
        }
        bore_at(0);
        for (x=[8,16]) translate([x,0,0]) cylinder(d=3.3,h=30,center=true);
    }
}
module washer() { difference() { cylinder(d=9,h=0.6); translate([0,0,-1]) cylinder(d=4.3,h=3); } }
module assembly() {
    color("silver") front_mount();
    for (i=[0:link_count-1])
        color(i%2==0 ? "lightsteelblue" : "silver") translate([pitch/2 + i*pitch,0,0]) link();
    color("silver") translate([link_count*pitch,0,0]) rear_mount();
    // pin locations (visual only, use real metal pins + shaft collars)
    for (i=[0:link_count]) color("gold")
        translate([i*pitch,0,0]) cylinder(d=pin_d,h=2*(ear_center+ear_thick/2)+1,center=true);
}
if (part=="assembly") assembly();
else if (part=="link") link();
else if (part=="front_mount") front_mount();
else if (part=="rear_mount") rear_mount();
else if (part=="washer") washer();
