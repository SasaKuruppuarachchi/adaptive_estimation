- To implement a tPC based adaptive estimator
- Deploy the system in Simulator and Real world hardware


# Objectives
## Phase 1 : Simulation
- Set up simulator in NVIDIA IsaacSIM #Sasa
- Replicate state estimation results of the paper.
	- KL as the baseline #Sasa 
	- tPC #Nick
- Off the shelf control. PID, LQR etc. #Sasa , #Nick 
- Evaluate systems adaptability with #Sasa , #Nick 
	- Changes in the physical parameters (e.g. gravity).
	- Changes in the system dynamics (mass of pendulum).
	- Unseen external disturbances.
	- Novel sensing methods.
# Phase 2 : Real world
- Construct the physical device #Sasa , #Nick 
- Evaluate the adaptability to sim2real gap for both KF and tPC. #Sasa , #Nick 