Why I think each trial failed:

scene_002_005: goal was too close to head, planner too conservative
scene_002_006: couldn't get around an obstacle
scene_003_001: failure to launch! start configuration too close to the ground.
scene_003_007: it basically reached the goal, but not quite because of self-intersection constraints I think. goal too close to self-intersection.
scene_004_002: basically reached the goal and then got stuck, really not sure why
scene_004_004: failure to launch! start configuration too close to robot head I think.
scene_004_006: failure to launch! start configuration too close to self-intersection I think.
scene_005_002: basically reached the goal, I think goal too close to the head
scene_006_004: failure to launch! I think start too close to base
scene_006_007: failure to launch! I think start too close to tower? not sure.
scene_007_007: got stuck... kind of a challenge to navigate between obstacle and head? this is a good one
scene_007_008: failure to launch! honestly no idea why. *** TAKE A CLOSER LOOK ***
scene_009_007: couldn't get around an obstacle, actually a pretty good trial
scene_010_005: basically reached goal and then got stuck... honestly no idea why *** TAKE A CLOSER LOOK ***
scene_010_008: failure to launch! honestly no idea why *** TAKE A CLOSER LOOK ***
scene_010_009: couldn't get around an obstacle, also a pretty good trial!

takeaways:
start or goal too close to body or ground: 8/16
failed to start moving for unknown reason: 2?/16
	update) scene 010_008 starts too close to obstacle 8, once removed trial is completed successfully... scene 007_008 starts too close to the tower.
couldn't get around an obstacle: 4/16
got stuck close to goal for unknown reason: 2?/16
	update) scene 004_002 joint 3 got past pi joint limit? scene 010_005 close to joint 4 limit? not sure why it fails here. goal is *very* close to joint limit