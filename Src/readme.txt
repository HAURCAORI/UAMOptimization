	sim_control.m
Basic PI controller implementation and simulation file.
Attitude and altitude control.
	
	sim_path_following.m
Based on sim_control.m, path following method is added.

	acs_anlaysis.m
ACS(attainable control set) analysis file.
ACS - Attainable force and moment set from feasible actuator set (Similar to torque envelope)
There are two cases for actuator configurations.
PNPNPN : Unable to flight even single motor failure case
PPNNPN : Able to flight in single motor failure case
(Refer : Fault-tolerant Control Allocation for Multirotor Helicopters using Parametric Programming)


