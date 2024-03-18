#! /usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim.srv import SetPen
from turtlesim.msg import Pose

import numpy as np
from scipy.optimize import minimize #optimization function from scipy librarry
from initDataSTSAMD import initDataSTSAMD #importing initial data
from consistentControl_obj import consistentControl_obj #objecive of the minimization
from consistentControl_grad import consistentControl_grad #gradient 
from RHstepSTSAMD import RHstepSTSAMD  # RDE data-- P matrix calculation
from gmtermSTSAMD import gmtermSTSAMD # game termination
from plotdataSTSAMD import plotdataSTSAMD #plot data fuunction 

rospy.init_node("controller_node")
def spawn_turtle(x,y,theta,agent_type):
    print("fuck")
    rospy.wait_for_service('/spawn')
    print("off")
    spawn_proxy=rospy.ServiceProxy('/spawn',Spawn)
    spawn_proxy(x=x,y=y,theta=theta,agent_type=agent_type,name="")

def set_pen(index,agent_type):
    turtle_setpen="/turtle"+str(index)+"/set_pen"
    rospy.wait_for_service(turtle_setpen)
    set_pen_proxy=rospy.ServiceProxy(turtle_setpen,SetPen)
    if agent_type==2:
        set_pen_proxy(r=255,g=0,b=0,width=2)
    elif agent_type==1:
        set_pen_proxy(r=0,g=255,b=0,width=2)
    else:
        set_pen_proxy(r=0,g=0,b=255,width=2)

def kill_turtle(index):
    rospy.wait_for_service("/kill")
    kill_proxy=rospy.ServiceProxy("/kill",Kill)
    name="turtle"+str(index)
    response=kill_proxy(name)
    return response


# Initialize nd, Active, and Players
#nd = 4 # choose the number of defenders 
nd=int(input("nd: ")) #choose the number of defenders
Active, Players, X0 = initDataSTSAMD(nd) # inital data function 
X0 = X0[3] #select initial index 
Players['X0'] = X0
Active['eventtime'] = {} #event time  dictionary
Active['event'] = {} #event dictionary
# Access Players' sensed and commd attributes
sensed = Players['sensed'] # defenders sensing redius 
commd = Players['commd'] # defenders communication redius
# Initialize xa0, z, xplayers, xa, xap, zfin, zp, and other variables
xa0 = Players['X0'][2 * (Players['Nd'] + 1):2 * (Players['Nd'] + 2)]
z = np.zeros((2 * (Players['Nd'] + 1), 1))
z[:] = Players['X0'][0:2 * (Players['Nd'] + 1)] - np.kron(np.ones((Players['Nd'] + 1, 1)), xa0)
#########
xplayers = np.zeros((2 * (Players['Nd'] + 2), 1))
xplayers[:] = Players['X0'] #inital position 
xa = np.zeros((2, 1))
xap = np.zeros((2, 1))
xa = xa0
xap = xa0
zfin = z[:]
zp = z[:]
RH = 0.1 * Players['T'] / Players['delta']
RHsteps = 100 # Receding Hosrizon steps 
I2 = np.eye(2)
xcnt = 0 # x counter 
itera = 0 # iterattion 
intloop = 0  #initial loop index 
finloop = RH * Players['delta'] #final loop index
pua = np.empty((2, 0))
put = np.empty((2, 0))

turtles=["/turtle"+str(i+1) for i in range(Active['N'])]
turtles_cmdvel=["/turtle"+str(i+1)+"/cmd_vel" for i in range(Active['N'])]

theta =0
for i in range(nd+2):
    xp=(xplayers[2*i])
    yp=(xplayers[2*i+1])
    if i<nd:
        #means a defender position
        agent_type=0
    elif i<nd+1:
        #means a target has spawned
        agent_type=1
    else:
        agent_type=2
    spawn_turtle(xp,yp,theta,agent_type)

for i in range(nd+2):
    if i<nd:
        agent_type=0
    elif i<nd+1:
        agent_type=1
    else:
        agent_type=2
    set_pen(i+1,agent_type)
publishers=[]
for i in range(Active['N']):
    pub=rospy.Publisher(turtles_cmdvel[i],Twist,queue_size=10)
    publishers.append(pub)

vel_cmd=Twist()
rate=rospy.Rate(100)
vy=[]  #See if this is needed
#########################################
for k in range(RHsteps):
    aclcnt = 0
    Players = RHstepSTSAMD(Active, Players) # RHstepSTSAMAD function 
    for m in np.arange(intloop, finloop, Players['delta']):
   
        kdvect = [] #feedback gain matrix of defenders -- KD --initialization
        flagsense = [] # Sensing flag matrix 
        flagcomm = [] # Communication flag matrix 
        Players['iMat'] = []
        ###############
        relpos = np.reshape(zp[:, xcnt][np.newaxis].T, (Active['maxNd'] + 1, 2)).T #relative position--reshaping 
        defpos = relpos[:, Active['Nd'][Active['epoch']]] # defender position 
        tarpos = relpos[:, -1][np.newaxis].T #target position
        adefenders = Active['Nd'][Active['epoch']] #active defendres 
        actnp = len(adefenders) + 1 # Active number of players 
        actnd = len(adefenders) #Active defenders 
        ##### Sorting P labels and all plabels ######
        Nd_epoch = Active['Nd'][Active['epoch']]  # Active defenders array
        maxNd = Active['maxNd']  #maximum number of defenders 
        maxNd_plus_1 = Active['maxNd'] + 1 
        S1 = [2 * i for i in Nd_epoch] + [2 * maxNd]
        S2 = [2 * i + 1 for i in Nd_epoch] + [2 * maxNd+1]
        plabels = sorted(S1 + S2)
        S3 = [2 * i  for i in Nd_epoch] + [2 * maxNd ] + [2 * maxNd_plus_1]
        S4 = [2 * i +1 for i in Nd_epoch] + [2 * maxNd + 1] + [2 * maxNd_plus_1 + 1]
        allplabels = sorted(S3 + S4) # sorting all the players labels 
        azp = zp[plabels, xcnt]
        azp = azp[np.newaxis].T
        Ind = np.eye(actnd)
        Inp = np.eye(actnp)
        mask = np.ones((actnd, actnd))
        ########### cpmmunocation and sensing flag matrix ###############
        for s in range(actnd):
            temp = np.column_stack((defpos, tarpos)) - np.kron(np.ones((1, actnp)), defpos[:,s][np.newaxis].T)
            temp[:, s] = defpos[:, s]
            tempsense = np.sqrt(np.sum(temp**2, axis=0)) - Players['sensed'][adefenders[s]] #sensing 
            tempcomm = np.sqrt(np.sum(temp**2, axis=0)) - Players['commd'][adefenders[s]]   #communication 
            flagsense.append(1 - 0.5 * (np.sign(tempsense) + 1))  #sensing flag 
            flagcomm.append(1 - 0.5 * (np.sign(tempcomm) + 1))   #communication flag 
            mask[s, s] = flagsense[s][s]
            
        flagsense = np.array(flagsense) # list to array -- flag sensing matrix 
        flagcomm = np.array(flagcomm) # list to array -- flag communication matrix     
        tempsc = np.double(np.logical_or(np.transpose(flagcomm[0:actnd, 0:actnd]),flagsense[0:actnd, 0:actnd]))
        flag = np.double(np.column_stack((np.logical_and(tempsc, mask).astype(int), flagsense[:, actnp-1][np.newaxis].T)))
  ############################ efenders Kd compuation ################################
        Players['iMat'] = np.zeros((actnd,2*actnp,2*actnp))
        #kdvect_array = np.array([])  #
        kdvect_list = [] 
        for s in range(actnd): 
            diag_flag_s = np.diag(flag[s, :])
            Inp_s_row = Inp[s,:]
            Inp_s_col = Inp[:,s][np.newaxis].T
            ones_actnp = np.ones((actnp, 1))
            mat_result = Inp - np.kron(Inp_s_row,ones_actnp-Inp_s_col)
            # Calculate Players['iMat']
            Players['iMat'][s,:,:] = np.kron(diag_flag_s @ mat_result, I2)
            # Calculate temp2
            temp2 = np.kron(Ind[s, :], I2) @ (-np.linalg.inv(Players['Rd']) @ Players['Bd'].T @ Players['Pd'][aclcnt,:, :])
            temp2 = temp2.reshape(-1, order='F')
            kdvect_list.append(temp2.ravel())
        #####
        kdvect_array = np.concatenate(kdvect_list, axis=0)  #kd matrix 
############################# Kd vect minimization ##########################
        kdvect_initial = kdvect_array  # # Initialization for kdvect
        options = {'disp': False,} # display on/off (verbose from the optimizer solver)
        # BFGS, Newton-CG, TNC, SLSQP
        kdvectopt = minimize(fun=consistentControl_obj,x0=kdvect_initial,args=(Players, aclcnt), method='BFGS', jac = consistentControl_grad, options=options)
        kdvect_opt = kdvectopt.x  # # Extract the optimized kdvect
        # Append results to pua and put on each xcnt iteration
        pua_result = -np.linalg.inv(Players['Ra']) @ np.transpose(Players['Ba']) @ Players['Pa'][aclcnt,:, :] @ azp
        put_result = -np.linalg.inv(Players['Rt']) @ np.transpose(Players['Bt']) @ Players['Pt'][aclcnt,:, :] @ azp
        # print("xcnt:----------------------------------------", xcnt)
        pua = np.hstack((pua, pua_result.reshape((2, 1)))) #attacker position 
        put = np.hstack((put, put_result.reshape((2, 1)))) #target position 
        
        #####################
        temp = 0
        Kd = np.zeros((actnd, 2, 2*actnp)) #initialization space for Kd 
        pud = np.zeros((2 , 1)) 
        for s in range(actnd):
            indx1 = 4 *s*actnp
            indx2 = 4 *(s+1)*actnp
            Kd[s,:,:] = np.reshape(kdvect_opt[indx1:indx2], (2 * actnp , 2)).T #reshaping Kd matrix 
            result = Kd[s, :, :].dot(Players['iMat'][s, :, :].dot(azp))
            pud = result.reshape((2,))
            temp = temp + Players['Bd'][:, 2 * s : 2*(s+1)] @ pud #matich perfectly
            

        temp = temp[np.newaxis].T
        
        ################################# Position vector of all the players after finding Kdvect ############
        xap_1 = xap[:, xcnt] + pua[:, xcnt] * Players['delta']
        xap_1 = xap_1[:, np.newaxis]  # Reshape to make it a column vector
        xap = np.hstack((xap, xap_1))  # Update xap for the next time step
        azpnxt = azp + Players['delta'] * (temp + Players['Bt'] @ put[:, xcnt][np.newaxis].T + Players['Ba'] @ pua[:, xcnt][np.newaxis].T)
        zp_1 = zp[:, xcnt]
        zp_1 = zp_1[:,np.newaxis]
        zp = np.hstack((zp,zp_1))
        ######
        azpnxt = azpnxt.flatten()  # Reshape azpnxt to (10,)
        zp[plabels, xcnt + 1] = azpnxt
        ###
        xplayers_1 = xplayers[:,xcnt][np.newaxis].T
        xplayers =  np.hstack((xplayers, xplayers_1))
        xplayers[:, xcnt + 1] = xplayers[:, xcnt]
        xapn_result= np.kron(np.ones((actnp, 1)), xap[:, xcnt + 1][np.newaxis].T)
        xapn_result1 = azpnxt[np.newaxis].T + xapn_result
        concatenated_xplayers =  np.concatenate([xapn_result1, xap[:, xcnt + 1][np.newaxis].T])
        xplayers[allplabels, xcnt + 1] = concatenated_xplayers.flatten()  # all the players position
        vels=(xplayers[:,xcnt+1]-xplayers[:,xcnt])*10

        for i in range(nd+2):
            xv=vels[2*i]
            yv=vels[2*i+1]
            vel_cmd.linear.x=xv
            vel_cmd.linear.y=yv
            publishers[i].publish(vel_cmd)
        rospy.sleep(0.1)
        for i in range(nd+2):
            vel_cmd.linear.x=0
            vel_cmd.linear.y=0
            publishers[i].publish(vel_cmd)
        
        ################################## Checking the game termination events ##################
        Active, Players = gmtermSTSAMD(azpnxt, Active, Players, m)

        if Active['eventflag'] == 1: 
            intloop = m + Players['delta']
            finloop = m + RH * Players['delta']
            itera += 1
            break

        if Active['TERMflag'] == 1:
            break
        else:
            aclcnt = aclcnt+1; # acl count increase 
            xcnt = xcnt+1; # x count increase 

    intloop = finloop + Players['delta']
    finloop = finloop + RH * Players['delta']
    #plotdataSTSAMD(xplayers, Active) ################# Plotting the data 

    if Active['TERMflag'] == 1:
        break