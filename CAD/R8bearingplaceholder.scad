// R8 placeholder: 1/2" bore x 1-1/8" OD x 5/16" wide
W    = 7.94;      // width
Ro   = 28.575/2;  // outer radius
Rb   = 12.7/2;    // bore radius
Rs   = 10.5;      
gap  = 0.4;       // total clearance, tune 0.3-0.5 
$fn  = 120;

path = [[Rs,0],[Rs,1.8],[Rs+1.5,W/2],[Rs,W-1.8],[Rs,W]];

module inner2d() polygon(concat([[Rb,0]], path, [[Rb,W]]));
module outer2d() polygon(concat([[Ro,0]], path, [[Ro,W]]));

rotate_extrude() offset(delta=-gap/2) inner2d();
rotate_extrude() offset(delta=-gap/2) outer2d();