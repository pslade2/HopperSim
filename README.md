# HopperSim
Matlab simulations for simple dynamic hopper

Each folder contains matlab files to run simulations of the hopper with different control methods.

To visualize, first run init_draw.m and then run main.m from a given folder. The main.m file can be re-run without having to run init_draw.m between subsequent runs. 

General file descriptions:
initdraw.m -- sets up the matlab plotter
draw.m -- called periodically to update the drawing output
main.m -- defines parameters for the system, simulation parameters such as duration, and calls the control.m file and the simulate.m file at each time step to get actions and propagate dynamics
control.m -- compute actions to be taken for a given controller
simulate.m -- update the dynamic model with new forces from control and propagate simple EOMs with an RK3 integration
