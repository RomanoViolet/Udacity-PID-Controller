import os
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import distance
import subprocess
import io
import re
import signal





# function definitions
def executeModel(currentDirectory,  K,  TotalFramesToObserve):

    # unpack the parameters
    Kp = K[0]
    Ki = K[1]
    Kd = K[2]
    

    # Get the parent to the current directory
    parentDirectory = os.path.dirname(currentDirectory);


    #cleanup the results folder
    pathToOldCSV = os.path.join(os.path.dirname(parentDirectory), "Results/ControllerTrace.csv")
    if(os.path.isfile(pathToOldCSV)):
        os.remove(pathToOldCSV)

    logfileName = os.path.join(os.path.join(parentDirectory, "Results/"), 'trace.txt')
    goodResultsLogFile = os.path.join(os.path.join(parentDirectory, "Results/"), 'GoodResults.log')

    #Counts the number of frames for the entire circuit
    nFrames = 0
    error = 0.0
    with open(logfileName, 'w') as f:

        cmd = [os.path.join(parentDirectory,  "Debug/PIDController"), str(Kp),  str(Ki),  str(Kd),  str(TotalFramesToObserve)]
        
        p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.STDOUT)
        PID_pid = p.pid
        for line in io.TextIOWrapper(p.stdout, encoding = "utf-8"):
            
            if((re.search('fatal', line, re.UNICODE)) or (re.search('nan', line, re.UNICODE)) or (re.search('Simulator aborted',  line,  re.UNICODE))):
                print ("\n"+"[FATAL]\tPID: " + line)
                error = re.search('.*Error:(.*)\s*and',  line,  re.UNICODE).group(1)
                
                
                f.write("[FATAL]\tPID" + line)
                os.kill(PID_pid, signal.SIGKILL)
                print("\n****** ABORTED ********\n\n\n\n\n")
                f.write("\n****** ABORTED ********\n\n\n\n\n")
                
            elif(re.search('Simulator Reset due to reaching the set number of frames with total error', line, re.UNICODE)):
                f.write("[END]\tPID" + line)
                print("PID Values: %f, %f, %f\n"%(Kp,  Ki,  Kd))
                error = re.search('.*error:(.*)\s*and', line, re.UNICODE).group(1)
                os.kill(PID_pid, signal.SIGKILL)
                
                
                
                # also dump it into a special file
                with open(goodResultsLogFile, 'a+') as g:
                    g.write("Error: %f \t PID Values: %f, %f, %f\n"%(float(error),  Kp,  Ki,  Kd))
                    
                g.close()
                
                print("\n****** Simulation Ended with Overall Total Error: %f ********\n\n\n\n\n" %(float(error)))
                f.write("\n****** Simulation Ended with Overall Total Error: %f ********\n\n\n\n\n" %(float(error)))
                
            else:
                #print ("PID: " + line)
                nFrames = nFrames + 1
                #f.write(line)

    f.close()
    return float(error)



def twiddleMine(currentDirectory,  TotalFramesToObserve, tol=0.001): 
    # Don't forget to call `make_robot` before every call of `run`!
    p = [0.2, 0.001, 0.1]
    dp = [0.01, 0.0001, 0.01]
    
    # Not Required anymore
    #robot = make_robot()
    
    
    error = executeModel(currentDirectory,  p,  TotalFramesToObserve)
    error = abs(error)
    #best_err = error
    nInter = 0
    while (sum(dp) > tol):
        nInter = nInter + 1
        print("Iteration: %d"%(nInter))

        
        for i in range(len(p)):
            # go up
            p[i] = p[i] + dp[i]
            
            # Not Required anymore
            #robot = make_robot()
            
            best_err = executeModel(currentDirectory,  p,  TotalFramesToObserve)
            if (abs(best_err) < error):
                error = abs(best_err)
                # going up worked. Go even more up next time
                dp[i] = dp[i] * 1.1
                
                #get to the next parameter
                continue
                
            # make a new robot
            # Not Required anymore
            #robot = make_robot()
            
            # going up did not work, so reset p[i]
            p[i] = p[i] - dp[i]
            
            # Attempt going up a little less
            p[i] = p[i] + 0.9*dp[i]
            best_err = executeModel(currentDirectory,  p,  TotalFramesToObserve)
            if (abs(best_err) < error):
                error = abs(best_err)
                # going up a little less worked. Set the dp to reflect the discovery
                dp[i] = dp[i] * 0.9
                
                #get to the next parameter
                continue
            
            # make a new robot
            # Not Required anymore
            #robot = make_robot()
            
            # reset previous p[i]
            p[i] = p[i] - 0.9*dp[i]
            
            # Going up does not work at all. Go down.
            p[i] = p[i] - dp[i]
            best_err = executeModel(currentDirectory,  p,  TotalFramesToObserve)
            if (abs(best_err) < error):
                error = abs(best_err)
                # going down worked. Go even more down next time
                dp[i] = dp[i] * 1.1
                
                #get to the next parameter
                continue
            
            
            # make a new robot
            # Not Required anymore
            #robot = make_robot()
            
            # reset previous p[i]
            p[i] = p[i] + dp[i]
            # Attempt going down a little less
            p[i] = p[i] - 0.9*dp[i]
            best_err = executeModel(currentDirectory,  p,  TotalFramesToObserve)
            if (abs(best_err) < error):
                error = abs(best_err)
                # going up a little less worked. Set the dp to reflect the discovery
                dp[i] = dp[i] * 0.9
                
                #get to the next parameter
                continue
            
            # reset previous p[i]
            p[i] = p[i] + 0.9*dp[i]
            
            #since there was no improvement either by going up or down,
            # the step size needs adjusting -- we will decrese the step size
            dp[i] = dp[i] * 0.9
            
            
        print ("p, i, d :%f,%f,%f"%(p[0],p[1],p[2]))
        # TODO: twiddle loop here
    
    return p, error


# The application expects all tunables to be provided from the command line

# Tunables
Kp = 1.0
Ki = 1.0
Kd = 1.0
TotalFramesToObserve = 50000

#Get current working directory
currentDirectory = os.getcwd()


twiddleMine(currentDirectory,  TotalFramesToObserve)


#executeModel(currentDirectory,  Kp,  Ki,  Kd,  TotalFramesToObserve)
