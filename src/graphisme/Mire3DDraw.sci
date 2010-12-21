

//---------------------------------------------//
// MIRE 3D DRAW
// date 12/2009
// author claire dune
//--------------------------------------------//
function Mire3DDraw5pts(P)
// cette fonction prepare un objet camera pour
// l'affichage
P1P2x = [P(1) P(4)]';
P1P2y = [P(2) P(5)]';
P1P2z = [P(3) P(6)]';

P2P5x = [P(4) P(13)]';
P2P5y = [P(5) P(14)]';
P2P5z = [P(6) P(15)]';

P5P4x = [P(13) P(10)]';
P5P4y = [P(14) P(11)]';
P5P4z = [P(15) P(12)]';

P1P4x = [P(1) P(10)]';
P1P4y = [P(2) P(11)]';
P1P4z = [P(3) P(12)]';

P1P3x = [P(1) P(7)]';
P1P3y = [P(2) P(8)]';
P1P3z = [P(3) P(9)]';

P2P3x = [P(4) P(7)]';
P2P3y = [P(5) P(8)]';
P2P3z = [P(6) P(9)]';

P5P3x = [P(13) P(7)]';
P5P3y = [P(14) P(8)]';
P5P3z = [P(15) P(9)]';

P3P4x = [P(7) P(10)]';
P3P4y = [P(8) P(11)]';
P3P4z = [P(9) P(12)]';

// P1->P2
plot3d (P1P2x, P1P2y, P1P2z);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;

// P2->P5
plot3d (P2P5x, P2P5y, P2P5z);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;

// P5->P4
plot3d (P5P4x, P5P4y, P5P4z);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;

// P4->P1
plot3d (P1P4x, P1P4y, P1P4z);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;

// P1->P3
plot3d (P1P3x, P1P3y, P1P3z);
hr1 = gce ();
hr1.thickness=1;
hr1.foreground=5;

// P2->P3
plot3d (P2P3x, P2P3y, P2P3z);
hr1 = gce ();
hr1.thickness=1;
hr1.foreground=5;

// P5->P3
plot3d (P5P3x, P5P3y, P5P3z);
hr1 = gce ();
hr1.thickness=1;
hr1.foreground=5;

// P3->P1
plot3d (P3P4x, P3P4y, P3P4z);
hr1 = gce ();
hr1.thickness=1;
hr1.foreground=5;
endfunction


function Mire3DDraw4pts(P)
// cette fonction prepare un objet camera pour
// l'affichage
P1P2x = [P(1) P(4)]';
P1P2y = [P(2) P(5)]';
P1P2z = [P(3) P(6)]';

P1P3x = [P(1) P(7)]';
P1P3y = [P(2) P(8)]';
P1P3z = [P(3) P(9)]';

P2P4x = [P(4) P(10)]';
P2P4y = [P(5) P(11)]';
P2P4z = [P(6) P(12)]';

P4P3x = [P(10) P(7)]';
P4P3y = [P(11) P(8)]';
P4P3z = [P(12) P(9)]';



// P1->P2
plot3d (P1P2x, P1P2y, P1P2z);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;

// P1->P3
plot3d (P1P3x, P1P3y, P1P3z);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;

// P2->P4
plot3d (P2P4x, P2P4y, P2P4z);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;

// P4->P3
plot3d (P4P3x, P4P3y, P4P3z);
hr1 = gce ();
hr1.thickness=3;
hr1.foreground=5;

endfunction

function IsLoadedMire3DDraw
disp('Yes ! Loaded Mire3DDraw')  
endfunction
