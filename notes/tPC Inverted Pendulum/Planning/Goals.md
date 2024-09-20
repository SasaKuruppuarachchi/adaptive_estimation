- To implement a tPC based adaptive estimator
- Deploy the system in Simulator and Real world hardware


# Objectives
## Phase 1:
- Set up simulator Nvidia Isaac sim (Sasa)
- Replicate state estimation results of the paper.
	- KL (Sasa)
	- tPC (Nick)
- Off the shelf control. PID, LQR etc.. (Sasa & Nick)
- Evaluate systems adaptability (learning transition and emission dynamics) with
	- changes in the physical parameters (e,g, gravity)
	- changes in the dynamical paremeters (mass of pendulum).
	-  novel sensing methods.
# Phase 2:
- Construct the physical device
- Evaluate the adaptability to sim2real gap for both KF and tPC.