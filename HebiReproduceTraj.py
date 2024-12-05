import hebi
import os
from time import sleep
import numpy as np
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('-d', '--data', help='folder for the dataset', type=str, default='A')
args = parser.parse_args()

lookup = hebi.Lookup()
# Give the Lookup process 2 seconds to discover modules
sleep(2)
print('Modules found on network:')
for entry in lookup.entrylist:
  print(f'{entry.family} | {entry.name}')

group = lookup.get_group_from_family('*')

# Reading the trajectories for both joints
joint1_pos = np.loadtxt(os.getcwd() + "/dataset/" + args.data + "/m1_pos.txt", dtype=float)
joint2_pos = np.loadtxt(os.getcwd() + "/dataset/" + args.data + "/m2_pos.txt", dtype=float)
joint1_vel = np.loadtxt(os.getcwd() + "/dataset/" + args.data + "/m1_vel.txt", dtype=float)
joint2_vel = np.loadtxt(os.getcwd() + "/dataset/" + args.data + "/m2_vel.txt", dtype=float)
joint1_tor = np.loadtxt(os.getcwd() + "/dataset/" + args.data + "/m1_torques.txt", dtype=float)
joint2_tor = np.loadtxt(os.getcwd() + "/dataset/" + args.data + "/m2_torques.txt", dtype=float)



t_step = 0.01
t_f = (len(joint1_pos) - 1)*t_step
num_joints = 2

# The times to reach each waypoint (in seconds)
time = np.arange(0.0, t_f+t_step*0.9, t_step)
positions = [joint1_pos, joint2_pos]
velocities = [joint1_vel, joint2_vel]
torques = [joint1_tor, joint2_tor]
# Define trajectory
trajectory = hebi.trajectory.create_trajectory(time, positions, velocities)

print(t_f)

# Follow the trajectory

cmd = hebi.GroupCommand(num_joints)
period = t_step
duration = trajectory.duration
pos_cmd = np.array(num_joints, dtype=np.float64)
vel_cmd = np.array(num_joints, dtype=np.float64)


t = 0.0

while (t < duration):
  pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
  cmd.position = pos_cmd
  cmd.velocity = vel_cmd
  index = int(t / t_step)
  
  # Ensure index is within bounds
  index = min(index, len(joint1_tor) - 1)
  
  # Assign sampled torques
  cmd.effort = [joint1_tor[index], joint2_tor[index]]

  group.send_command(cmd)

  t = t + period
  sleep(period)
  if(t > duration - period):
    t = 0.0
