import sys
import os

# Add the subdirectory to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'bin'))
from bin import prisma_walker

import math
import time
from ppo import module as ppo_module
import numpy as np
import torch
import datetime
import argparse
import pickle 
import matplotlib 
from threading import Thread
matplotlib.use('agg')
from matplotlib import pyplot as plt


net_size = [256, 128]
num_envs = 1


motorTorque_x = []
q_1 = []
q_2 = []

pTarge_x = []
pTarge_y = []

dotq_1 = []
dotq_2 = []

#simulation_time = np.arange(0, max_steps/100, 0.01, dtype='float32') 
simulation_time = []
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."
plot_dir = task_path + "/plots/"

if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)

def runNN(max_steps):
    

    obs = np.zeros([ob_dim], dtype=np.float32)
    #ob_double = prisma_walker.EigenVectorXd(obs.flatten().astype(np.float64))


    if (weight_path == ""):# or ( flag == 0):
        print("Can't find trained weight, please provide a trained weight with --weight switch\n")
    else:
        print("Loaded weight from {}\n".format(weight_path))
        env.reset()

        start_step_id = 0

        print("Visualizing and evaluating the policy: ", weight_path)
        loaded_graph = ppo_module.MLP(net_size, torch.nn.LeakyReLU, ob_dim, act_dim, num_envs)
        loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

        #env.command_vel(*args.velocity) #l'asterisco * serve a transformare [1,1,1] in 1,1,1"""

        #load_scaling(weight_dir, int(iteration_number))

        current_time= 0
 
        """traj_x = []
        with open("/home/claudio/raisim_ws/raisimlib/raisimGymTorch/raisimGymTorch/env/envs/prisma_walker/trajectory_motor1.txt") as file:
            traj_x = [line.strip() for line in file]

        traj_y = []
        with open("/home/claudio/raisim_ws/raisimlib/raisimGymTorch/raisimGymTorch/env/envs/prisma_walker/trajectory_motor2.txt") as file:
            traj_y = [line.strip() for line in file]"""

        #obs_l = torch.from_numpy(obs)
        for step in range(max_steps):
            if current_time == 0:
                time.sleep(1)
            else:
                time.sleep(0.01)
            if step == 0:
                time.sleep(1)

            env.observe(obs) #if the shape or the type is not correct, it raises an error like that in whihc you wrote env.ENVIRONMENT instead of env.ENVIRONMANE()  

            action_ll = loaded_graph.architecture(torch.from_numpy(obs).cpu())

            env.step(action_ll.cpu().detach().numpy())

            q_1.append(env.getPositions()[0])
            q_2.append(env.getPositions()[1])

            dotq_1.append(env.getVelocities()[0])
            dotq_2.append(env.getVelocities()[1])
            
            motorTorque_x.append(env.getMotorTorques()[0])
            motorTorque_x.append(env.getMotorTorques()[1])

            pTarge_x.append(env.getReferences()[0])
            pTarge_y.append(env.getReferences()[1])

            current_time = current_time + 0.01
            simulation_time.append(current_time)
            #time.sleep(0.01)
            if step == max_steps - 1:
                print('----------------------------------------------------')
                print('{:<40} {:>6}'.format("time elapsed [sec]: ", '{:6.4f}'.format((step + 1 - start_step_id) * 0.01)))
                print('----------------------------------------------------\n')
                start_step_id = step + 1


        """SAVE INTO A FILE
        with open(r'/home/claudio/raisim_ws/raisimlib/raisimGymTorch/raisimGymTorch/env/envs/prisma_walker/trajectory_motor1.txt', 'w') as fp:
            for item in pTarge_x:
                fp.write("%s\n" % item)
        
        with open(r'/home/claudio/raisim_ws/raisimlib/raisimGymTorch/raisimGymTorch/env/envs/prisma_walker/trajectory_motor2.txt', 'w') as fp:
            for item in pTarge_y:
                fp.write("%s\n" % item)"""


    def load_scaling(dir_name, iteration, count=1e5):
        mean_file_name = dir_name + "/mean" + str(iteration) + ".csv"
        var_file_name = dir_name + "/var" + str(iteration) + ".csv"
        
        mean = np.loadtxt(mean_file_name, dtype=np.float32)
        var = np.loadtxt(var_file_name, dtype=np.float32)
        #wrapper.setObStatistics(self.mean, self.var, self.count)


def plotVariables(max_steps):

    plotDatas(simulation_time, actualTorque_x, motorTorque_x, "implicitly integrated torque", "explitly integrated torque", "joints torques", "$N/m$")
    plotDatas(simulation_time, q_1, pTarge_x, "$q_1$", "$\hat{q}_{1}$", "m1 joint positions", "rad")
    plotDatas(simulation_time, q_2, pTarge_y, "$q_2$", "$\hat{q}_{2}$", "m2 joint positions", "rad")
    plotDatas(simulation_time, dotq_1, dotq_2, "$\dot{q}_1$", "$\dot{q}_2$", "joint velocities", "rad/s")
    plotDatas(simulation_time, ddotq_1, ddotq_1, "$\ddot{q}_1$", "$\ddot{q}_2$", "joint accelerations", "rad/$s^2$")

def plotDatas(t, x, y, label1, label2, title, ylabel):
  
    plt.figure()
    plt.plot(t, x, label=label1)
    plt.plot(t, y, label=label2)
    plt.title(title)
    plt.xlabel('time')
    plt.ylabel(ylabel)
    plt.grid()
    plt.legend()
    plt.savefig(plot_dir + title + ".png")



if __name__ == '__main__':
        # configuration
    parser = argparse.ArgumentParser()
    parser.add_argument('-w', '--weight', help='trained weight path', type=str, default='')
    args = parser.parse_args()

    task_name = "prisma_walker_locomotion"
    task_path = os.path.dirname(os.path.realpath(__file__))

    env = prisma_walker.ENVIRONMENT() #forgetting parenthesis here were causing errors

    # shortcuts
    ob_dim = env.num_obs()
    act_dim = env.num_actions()
    control_dt = 0.01

    weight_path = "/home/claudio/raisim_ws/raisimlib/raisimGymTorch/data/prisma_walker/str-Not-priv/full_2500.pt"
    iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0] #rsplit splitta da destra. Fermati al '/' e dividi in 2. Poi prendi il secondo elemento del vettore, cioè quello che sta a destra
    weight_dir = weight_path.rsplit('/', 1)[0] + '/'

    #duration in ms of the simulation
    max_steps = 4000
    nnthread = Thread(target = runNN, args= (max_steps, ))
    plotThread = Thread(target = plotVariables, args= (max_steps, ))

    try: 
        nnthread.start()
        nnthread.join()
    except KeyboardInterrupt:
        plotThread.start()
        plotThread.join()
