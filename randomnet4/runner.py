# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/> #66 this is the yellow phase, where the deceleration is required 
#        <phase duration="31" state="rGrG"/> 
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>


#66 traci.trafficlight directly calls the function in traci
#66 need to understand the phase is subset of program 
#66 We let the server simulate one simulation step, read the induction loop and switch the phase of the traffic light until 
# the end is reached where no vehicle exists on the road anymore. If we find a vehicle on the induction loop the phase is 
# switched such that the north south direction gets green. If no vehicle is on the detector, and we are not already in the 
# process of switching (so EW has still green), we try to keep this phase by simply setting it again. At the end we close the connection.


from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import numpy as np

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


TL=['A','B','C','D','E','F','G','H','I','J']


Num_detectors = 360
throughput = []
num_all = []

# you need to specify the traffic light id 
def run():
    """execute the TraCI control loop"""
    step = 0
    buff = 2000 # change the buffer if you want
    for i in range(buff):
        traci.simulationStep() # this is to give the simulation an initialization
        sum=0
        for j in range(Num_detectors):
            # print("get the information from detector",str(j))
            sum += traci.inductionloop.getLastStepVehicleNumber(str(j))
        throughput.append(sum)
        num_all.append(traci.simulation.getMinExpectedNumber())
        # print("the number of vehicles in the scenario",traci.simulation.getMinExpectedNumber())
 
    print("after the buffer time, we start control the signals using SQF rule")
    file = open("/home/hao/Documents/traci_test/randomnet4/flow.txt",'w')
    file_density = open("/home/hao/Documents/traci_test/randomnet4/density.txt",'w')

    # edgeSet = traci.edge.getIDList()
    # print("the edge list is", edgeSet)

    NSphase = 0
    EWphase = 2
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("G6", NSphase) # 66 switch to the pahse with given index in the list of all phases for the current program
    while traci.simulation.getMinExpectedNumber() > 0: # run the simulation until there is no vehicles on the network
        traci.simulationStep() # this means let the simulation go forward for one step 
        # traci.simulationStep() # this means let the simulation go forward for one step 
        # traci.simulationStep() # At least let each phase last three steps
        ################## we need to save the information for all the loop detectors###############
        step += 1

        sum=0
        for i in range(Num_detectors):
            sum += traci.inductionloop.getLastStepVehicleNumber(str(i))
        throughput.append(sum) # the total number of vehicles passing all the loop detectors
        file.write(str(sum)+',')
        file.flush()
        # num_veh = traci.simulation.getMinExpectedNumber()# this is to get the number of vehicles in the network
        num_veh = traci.vehicle.getIDCount()
        print("the total number of vehicles is ",num_veh)
        num_all.append(num_veh)
        # print("the number of vehicles in the scenario",traci.simulation.getMinExpectedNumber())
        file_density.write(str(num_veh)+',')
        file_density.flush()
        # print("the number of vehicles for the specific loop detector is",traci.inductionloop.getLastStepVehicleNumber(str(200)))
        ############################### here we should have a loop for every signal ##################################
        # have a list for all junction nodes
        for i in range(1,9):
            for j in range(1,9):
                    NSdelay = traci.edge.getWaitingTime(TL[i]+str(j-1)+TL[i]+str(j))+ traci.edge.getWaitingTime(TL[i]+str(j+1)+TL[i]+str(j))
                    EWdelay = traci.edge.getWaitingTime(TL[i+1]+str(j)+TL[i]+str(j))+ traci.edge.getWaitingTime(TL[i-1]+str(j)+TL[i]+str(j))
                    if EWdelay > NSdelay: # if the NS direction has less delay (shorter queue) 
                        traci.trafficlight.setPhase(TL[i]+str(j), NSphase)
                    else:
                        # otherwise try to keep green for EW    
                        traci.trafficlight.setPhase(TL[i]+str(j), EWphase)
    traci.close()
    sys.stdout.flush()



# this is the main entry point of this script
if __name__ == "__main__":

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start(["/home/hao/sumo_binaries/bin/sumo-gui", "-c", "/home/hao/Documents/traci_test/randomnet4/random.sumo.cfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()


