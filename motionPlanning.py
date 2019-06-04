#!/usr/bin/env python
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.integrate import odeint

# Function to plot a cube
def plot_linear_cube(ax, x, y, z, dx, dy, dz, color):
    xx = [x, x, x+dx, x+dx, x]
    yy = [y, y+dy, y+dy, y, y]
    kwargs = {'alpha': 1, 'color': color}
    ax.plot3D(xx, yy, [z]*5, **kwargs)
    ax.plot3D(xx, yy, [z+dz]*5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z+dz], **kwargs)
    ax.plot3D([x, x], [y+dy, y+dy], [z, z+dz], **kwargs)
    ax.plot3D([x+dx, x+dx], [y+dy, y+dy], [z, z+dz], **kwargs)
    ax.plot3D([x+dx, x+dx], [y, y], [z, z+dz], **kwargs)
    

def get_Normal(p1,p2,p3):    
    a = (p2[1] - p1[1])*(p3[2] - p1[2]) - (p2[2] - p1[2])*(p3[1] - p1[1]) 
    b = (p2[2] - p1[2])*(p3[0] - p1[0]) - (p2[0] - p1[0])*(p3[2] - p1[2]) 
    c = (p2[0] - p1[0])*(p3[1] - p1[1]) - (p2[1] - p1[1])*(p3[0] - p1[0])
    return [a,b,c]


# Get point of intersection
def line2plane(planeVector,planePoint,lineVector,linePoint):
    # planeVector: normal of the plane
    # lineVector: direction of the line
    vp1 = planeVector[0]
    vp2 = planeVector[1]
    vp3 = planeVector[2]
    n1 = planePoint[0]
    n2 = planePoint[1]
    n3 = planePoint[2]
    v1 = lineVector[0]
    v2 = lineVector[1]
    v3 = lineVector[2]
    m1 = linePoint[0]
    m2 = linePoint[1]
    m3 = linePoint[2]
    
    # Determine whether the line is perpendicular to normal of the plane
    vpt = v1 * vp1 + v2 * vp2 + v3 * vp3
    if (vpt == 0):
        x = 10000
        y = 10000
        z = 10000
    else:
        t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt
        x = m1 + v1 * t
        y = m2 + v2 * t
        z = m3 + v3 * t
    return [x,y,z]


def weight_Generation(p1,p2,plane):
    lineVector = np.array([p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]])
    planeVector = plane[0]
    pa = plane[1]
    pb = plane[2]
    planePoint = pa
    interpoint = line2plane(planeVector,planePoint,lineVector,p1)
    # whether the point is in line section
    index = 0
    if interpoint[0]>min(p1[0],p2[0]) and interpoint[0]<max(p1[0],p2[0]):
        # whether the point is in plane of the osbtables
        for i in range(3):
            if planeVector[i] == 0:
                if interpoint[i]>min(pa[i],pb[i]) and interpoint[i]<max(pb[i],pa[i]):
                    index = index+1
        if index == 2:
            weight = 10000
        else: weight = np.linalg.norm(np.array([p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]]))
    else: weight = np.linalg.norm(np.array([p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]]))
    return weight    
    
class cube:
    def __init__(self, origin,size):
        
        x = origin[0]
        y = origin[1]
        z = origin[2]
        l = size[0]
        w = size[1]
        h = size[2]
        # Define vertices of cubiod        
        self.p1 = [x,y,z]
        self.p2 = [x+l,y,z]
        self.p3 = [x,y+w,z]
        self.p4 = [x,y,z+h]
        self.p5 = [x+l,y+w,z]
        self.p6 = [x+l,y,z+h]
        self.p7 = [x,y+w,z+h]
        self.p8 = [x+l,y+w,z+h]
        self.m1 = [x+l,y+w/2,z]
        self.m2 = [x+l,y+w/2,z+h]
        self.m3 = [x,y+w/2,z]
        
        
    def planeGeneration(self):
        plane = [] # 7 element, every element is a 3+6_d vector [normal+diagonal nodes]
        plane.append(np.array([get_Normal(self.p1,self.p2,self.p4),self.p2,self.p4])) 
        plane.append(np.array([get_Normal(self.p2,self.p5,self.p6),self.p5,self.p6])) 
        plane.append(np.array([get_Normal(self.p4,self.p6,self.p7),self.p6,self.p7])) 
        plane.append(np.array([get_Normal(self.p3,self.p5,self.p8),self.p3,self.p8])) 
        plane.append(np.array([get_Normal(self.p1,self.p2,self.p3),self.p2,self.p3]))
        plane.append(np.array([get_Normal(self.p1,self.p4,self.p3),self.p4,self.p3]))
        plane.append(np.array([get_Normal(self.m1,self.m2,self.m3),self.m2,self.m3]))

        return plane


    def verticesGeneration(self):
        vertices = np.array([self.p1,self.p2,self.p3,self.p4,self.p5,self.p6,self.p7,self.p8])
        return vertices         


def takeFirst(elem):
    return elem[0]


class Node:
    def __init__(self,p_cost,h_cost,t_cost,posi,ind):
        self.past_cost = p_cost
        self.heur_cost = h_cost
        self.total_cost = t_cost
        self.parent = [] # Elements of that list should be node, which will be set in Grid_map
        self.neighbor = [] # Elements of that list should be (node,weight)
        self.position = np.array(posi)
        self.index = ind


class Astar:
    def __init__(self, Start_Goal_point,graph):
        '''  global variable here  '''
        global running_time
        
        # Start_Goal_point information [3*1,3*1]
        self.data = Start_Goal_point
        self.openset = [] # (node_total_cost,node)
        self.closedset = []
        self.optimal_path = []
        
        self.info = str(graph)
        
        running_time.append(time.time())
        self.map_Generate()
        running_time.append(time.time())
        self.Astar_run()
        running_time.append(time.time())
        self.optimal_plot()
        
    def map_Generate(self):
        # graph: map information [map_size, #_obstacles, location_of_obstacles, size_of_obstacles]
        #[3*1,1,6*1,6*1...]
        
        # Size of quadrotor
        quadrotor_x = 0.92
        quadrotor_y = 0.92
        quadrotor_z = 0.29
        extend_xy = quadrotor_x**(0.5)
        extend_z = (quadrotor_x**2+quadrotor_z**2)**(0.5)
        
        # Generate obstacle expansion graph
        fig = plt.figure(1)
        self.ax = Axes3D(fig)   
        self.planeSet = [] # plane for test
        self.verticesSet = [] # position of nodes
        
        map_info = list(map(int, self.info.split()))
        map_size = np.array([map_info[0],map_info[1],map_info[2]])
        plot_linear_cube(self.ax,0,0,0, map_size[0], map_size[1], map_size[2], 'black')
        self.obs_num = map_info[3]
        obs_size = np.zeros((3))
        obs_loc = np.zeros((3))
       
        for i in range(self.obs_num): 
            obs_loc = np.array([map_info[6*i+4],map_info[6*i+5],map_info[6*i+6]])
            obs_size = np.array([map_info[6*i+7],map_info[6*i+8],map_info[6*i+9]])  
            self.obs_ext_size = np.array([map_info[6*i+7]+extend_xy,map_info[6*i+8]+extend_xy,map_info[6*i+9]+extend_z])
            self.obs_ext_loc = np.array([map_info[6*i+4]-extend_xy/2,map_info[6*i+5]-extend_xy/2,map_info[6*i+6]-extend_z/2])
            cubeSet = cube(self.obs_ext_loc,self.obs_ext_size)
            self.planeSet.append(cubeSet.planeGeneration())
            self.verticesSet.append(cubeSet.verticesGeneration())
            
            plot_linear_cube(self.ax, obs_loc[0], obs_loc[1], obs_loc[2], obs_size[0], obs_size[1], obs_size[2], 'red')
            plot_linear_cube(self.ax, self.obs_ext_loc[0], self.obs_ext_loc[1], self.obs_ext_loc[2], self.obs_ext_size[0], self.obs_ext_size[1], self.obs_ext_size[2],'blue')

        m = 1
        self.node_Initialization()
        
        
    def node_Initialization(self): 
        N = 2+8*self.obs_num # number of nodes
        M = 7*self.obs_num # number of plane
        self.Nodelist = []
        Heuri = np.zeros((N))
        Weight = np.zeros((N,N))
        current_weight = np.zeros((M))
        k = 0 # index for iterition
        start_position = np.array([self.data[0],self.data[1],self.data[2]])
        goal_position = np.array([self.data[4],self.data[5],self.data[6]])
        Heuri[k] = np.linalg.norm(start_position-goal_position)
        
        
        # Node Initialization
        self.s_point = Node(0,Heuri[k],Heuri[k],start_position,k+1)
        self.Nodelist.append(self.s_point)
        for i in range(self.obs_num):
            for j in range(8): 
                k = k+1
                position_init = self.verticesSet[i][j]
                Heuri[k] = np.linalg.norm(position_init-goal_position)
                self.Nodelist.append(Node(10000,Heuri[k],10000+Heuri[k],position_init,k+1))
        self.g_point = Node(10000,0,10000,goal_position,k+2)
        self.Nodelist.append(self.g_point) 
        
        # Neighbor and weight initialization
        for i in range(N):
            for j in range(N):
                # Dertermine whether the line between two nodes go across obstacles
                for m in range(M): 
                    current_weight[m] = weight_Generation(self.Nodelist[i].position,self.Nodelist[j].position,self.planeSet[m//7][m%7])
                Weight[i][j] = max(current_weight)
                if Weight[i][j]<9999:
                    self.Nodelist[i].neighbor.append((self.Nodelist[j],Weight[i][j]))

              
    def Astar_run(self):
        '''  global variable here  '''
        global waypoints    
        self.openset.append((self.Nodelist[0].total_cost,self.Nodelist[0]))
        
        while self.openset:
            current = self.openset[0] # Get the first element in openset
            current_node = current[1] # Get the node
            # Check whether we have already arrived at goal point
            if current_node.index == self.g_point.index:
                self.optimal_path.append(current_node.position)
                current_parent = current_node.parent[0]
                while current_parent.index !=self.s_point.index:
                    self.optimal_path.append(current_parent.position)
                    current_parent = current_parent.parent[0]
                self.optimal_path.append(self.s_point.position)
                self.optimal_path.reverse()
                print("The waypoints are the following:")
                print(np.array(self.optimal_path))
                waypoints = self.optimal_path
                break
                
            #neighborset = current_node.neighbor
            N = len(current_node.neighbor)
            for i in range(N):
                current_neighbor = current_node.neighbor[i]
                neighbor_node = current_neighbor[0] # Get the node
                if self.closedset.count(neighbor_node) == 0:                    
                    temporary_cost = current_neighbor[1]+current_node.past_cost # weight+current's cost
                    if temporary_cost < neighbor_node.past_cost:
                        neighbor_node.past_cost = temporary_cost
                        neighbor_node.total_cost = neighbor_node.past_cost+neighbor_node.heur_cost
                        neighbor_node.parent = [current_node]
                        self.openset.append((neighbor_node.total_cost,neighbor_node))
            del self.openset[0]
            self.openset.sort(key = takeFirst)
            self.closedset.append(current_node)
        if not self.openset:
            print('No path exist')
            
    def optimal_plot(self):
        path = np.transpose(self.optimal_path)
        self.ax.plot3D(path[0],path[1],path[2],'g*-')
        #plt.show()


class Model3D:
	def __init__(self, data, waypoints):
		'''  global variable here  '''
		global running_time
		q0 = np.array([data[0], data[1], data[2], data[3]])
		qh = np.array([data[4], data[5], data[6], data[7]])

		print(' ')
		print('3D model is running... Please wait for a moment')
		print(' ')
		
		self.getReference(q0, qh, waypoints) # generate reference trajectory
		running_time.append(time.time())

		self.X = np.zeros((self.N,12))  # index 0-11
		self.U = np.zeros((self.N,4))   # index 0-3

		self.X[0,:] = self.Xr[0,:]    # set init state to reference state
		self.U[0,:] = np.array([0,0,0,0])

		# simulation starts
		for i in range(1,self.N):
			# calculate control input
			self.positionController(step=i)
			self.attitudeController(step=i)

			# span for next time step
			tspan = [self.time[i-1],self.time[i]] 

			# solve for next step
			output = odeint(self.model, self.X[i-1,:], tspan, args=(self.U[i-1,:],)) 
			self.X[i,:] = output[1,:]  # take the value at time i and drop that at time i-1

		running_time.append(time.time())
		print('time for (1): ' + str(running_time[1]-running_time[0]) )
		print('time for (2): ' + str(running_time[2]-running_time[1]) )
		print('time for (3): ' + str(running_time[3]-running_time[2]) )
		print('time for (4) and (5): ' + str(running_time[4]-running_time[3]) )
		print('total time: ' + str(running_time[4]-running_time[0]) )


		# plot results - you can zoom in to see details in these figures
		plt.figure(2)
		plt.plot(self.time,self.Xr[:,0],'r-',label='xr(t)')
		plt.plot(self.time,self.Xr[:,1],'g-',label='yr(t)')
		plt.plot(self.time,self.Xr[:,2],'b-',label='zr(t)')
		plt.plot(self.time,self.Xr[:,8],'k-',label='psir(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('reference trajectory')

		plt.figure(3)
		plt.plot(self.time,self.U[:,0],'m-',label='F(t)')
		plt.plot(self.time,self.U[:,1],'r-',label='M1(t)')
		plt.plot(self.time,self.U[:,2],'g-',label='M2(t)')
		plt.plot(self.time,self.U[:,3],'b-',label='M3(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('control input')

		plt.figure(4)
		plt.plot(self.time,self.X[:,0],'r-',label='x(t)')
		plt.plot(self.time,self.X[:,1],'g-',label='y(t)')
		plt.plot(self.time,self.X[:,2],'b-',label='z(t)')
		plt.plot(self.time,self.X[:,3],'r--',label='x`(t)')
		plt.plot(self.time,self.X[:,4],'g--',label='y`(t)')
		plt.plot(self.time,self.X[:,5],'b--',label='z`(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('translational state evolution')

		plt.figure(5)
		plt.plot(self.time,self.X[:,6],'r-',label='phi(t)')
		plt.plot(self.time,self.X[:,7],'g-',label='theta(t)')
		plt.plot(self.time,self.X[:,8],'b-',label='psi(t)')
		plt.plot(self.time,self.X[:,9],'r--',label='phi`(t)')
		plt.plot(self.time,self.X[:,10],'g--',label='theta`(t)')
		plt.plot(self.time,self.X[:,11],'b--',label='psi`(t)')
		plt.ylabel('values')
		plt.xlabel('time')
		plt.legend(loc='best')
		plt.title('rotational state evolution')

		plt.show()


	# 3d quadrotor model to be integrated
	def model(self,x,t,u):
		# x[0] - x ; x[1] - y ;  x[2] - z
		# x[3] - x`; x[4] - y`;  x[5] - z`
		# x[6] - phi;      x[7] - theta;     x[8] - psi
		# x[9] - omega_x; x[10] - omega_y;  x[11] - omega_z
		g = 9.8
		m = 0.03

		# compute r_dot
		r_dot = np.matrix(x[3:6]).transpose()

		# compute Omega_dot
		omega = np.matrix(x[9:12]).transpose()
		phi = x[6]
		theta = x[7]
		psi = x[8]
		R0 = np.matrix([[1, 0 ,-np.sin(theta)],
					    [0 ,np.cos(phi),np.cos(theta)*np.sin(phi)],
					    [0 ,-np.sin(phi),np.cos(theta)*np.cos(phi)]])
		R0_inv = np.linalg.inv(R0)
		Omega_dot = R0_inv*omega

		# compute r_ddot
		Rx_phi = np.matrix([[1, 0, 0], [0, np.cos(phi), np.sin(phi)], [0, -np.sin(phi), np.cos(phi)]])
		Ry_theta = np.matrix([[np.cos(theta), 0, -np.sin(theta)], [0, 1, 0], [np.sin(theta), 0, np.cos(theta)]])
		Rz_psi = np.matrix([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
		Rsb_T = Rx_phi*Ry_theta*Rz_psi
		Rsb = np.transpose(Rsb_T)
		g_vec = np.matrix([[0],[0],[-g]])
		F_vec = np.matrix([[0],[0],[u[0]/m]])
		r_ddot = g_vec + Rsb*F_vec

		# compute omega_dot
		Ib = np.matrix([[1.43,0,0],[0,1.43,0], [0,0,2.89]])*10**(-5)
		Ib_inv = np.linalg.inv(Ib)
		Moment = np.matrix(u[1:4]).transpose()
		omega_dot = Ib_inv*(Moment - np.cross(omega,Ib*omega, axis=0) )

		# put everything together
		dxdt = np.array([r_dot, r_ddot, Omega_dot, omega_dot]) # 12 dimension
		dxdt = np.reshape(dxdt,12)
		return dxdt


	def getReference(self, q0, qh, waypoints):
		waypoint_num = len(waypoints)
		waypoint_nparray = np.array(waypoints)
		waypoint_vectors = list()
		distance = list()
		time_step = list()
		total_time_step = 0
		for i in range(waypoint_num-1):
			waypoint_vectors.append(waypoint_nparray[i+1]-waypoint_nparray[i])
			distance.append(np.linalg.norm(waypoint_vectors[i]))
			time_step.append(int(distance[i]/2.0)+1) # max speed is 2m/s
			total_time_step += time_step[i]
		
		self.N = total_time_step*10 + 1  # steps, int
		self.time = np.linspace(0,total_time_step,self.N) # in every 0.1 second
		self.Xr = np.zeros((self.N,12))

		psi0 = q0[3]
		psih = qh[3]
		psi_vector = psih - psi0
		for step in range(self.N):
			self.Xr[step,8] = psi0 + psi_vector*(float(step)/self.N)

		past_time = 0
		for i in range(waypoint_num-1):
			init = waypoint_nparray[i]
			vector = waypoint_vectors[i]
			N = 10*time_step[i]
			for step in range(past_time, past_time + N):
				self.Xr[step,0:3] = init + vector*(float(step-past_time)/N)
				self.Xr[step,3:6] = vector * (2.0 / np.linalg.norm(vector) ) # max 2m/s speed
			past_time += N

	def positionController(self, step):
		m = 0.03
		g = 9.8
		kdx = 2
		kpx = 1
		kdy = 2
		kpy = 1
		kdz = 2
		kpz = 1

		# slicing 0:6 takes 0,1,2,3,4,5
		R = self.X[step-1, 0:6] # translational state feedback
		Omega = self.X[step-1,6:12] # rotational state feedback

		x = self.Xr[step-1,0] # reference
		y = self.Xr[step-1,1] # reference
		z = self.Xr[step-1,2] # reference
		x_dot = self.Xr[step-1,3] # reference
		y_dot = self.Xr[step-1,4] # reference
		z_dot = self.Xr[step-1,5] # reference
		psi = self.Xr[step-1,8] # reference

		x_control = kdx*(x_dot-R[3])+kpx*(x-R[0])
		y_control = kdy*(y_dot-R[4])+kpy*(y-R[1])
		x_control_hat = np.cos(Omega[2])*x_control+np.sin(Omega[2])*y_control
		y_control_hat = -np.sin(Omega[2])*x_control+np.cos(Omega[2])*y_control
		z_control = kdz*(z_dot-R[5])+kpz*(z-R[2])
		self.F = np.array([ m*z_control/(np.cos(Omega[1])*np.cos(Omega[0]))+m*g ])
		self.U[step-1,0] = self.F

		phi_des = np.arcsin(-y_control_hat/g)
		theta_des = np.arcsin(x_control_hat/(g*np.cos(Omega[0])))     # np.arcsin
		psi_des = psi
		self.Omega_des = np.array([phi_des, theta_des, psi_des])


	def attitudeController(self, step):
		kdphi = 8
		kpphi = 16
		kdtheta = 8
		kptheta = 16
		kdpsi = 2
		kppsi = 1

		Omega = np.matrix(self.X[step-1,6:9]).transpose()
		omega = np.matrix(self.X[step-1,9:12]).transpose()
		phi = Omega[0]
		theta = Omega[1]
		psi = Omega[2]
		kd = np.matrix([[kdphi,0,0], [0,kdtheta,0], [0,0,kdpsi]], dtype='float')
		kp = np.matrix([[kpphi,0,0], [0,kptheta,0], [0,0,kppsi]], dtype='float')
		I = np.matrix([[1.43,0,0], [0,1.43,0], [0,0,2.89]])*10**(-5)

		R0 = np.matrix([[1, 0 ,-np.sin(theta)],
						[0 ,np.cos(phi),np.cos(theta)*np.sin(phi)],
						[0 ,-np.sin(phi),np.cos(theta)*np.cos(phi)]], dtype='float')
		R0_inv = np.linalg.inv(R0)
		omega_bracket = np.matrix([[0,-omega[2],omega[1]],[omega[2],0,-omega[0]],[-omega[1],omega[0],0]])
		omega_dot = omega_bracket*I*omega
		Omega_des = np.matrix(self.Omega_des).transpose()

		self.M = I*R0*(kd*(-R0_inv*omega)+kp*(Omega_des-Omega))+omega_dot
		self.U[step-1,1:4] = np.array(self.M.transpose())


if __name__ == '__main__':
    
    print('')
    print('Please set the initial and goal configurations [q0,qh] = [x0 y0 z0 yaw0 xh yh zh yawh]')
    init_goal_s = raw_input()
    init_goal = list(map(float, init_goal_s.split()))
    assert len(init_goal) == 8
    print('Please set the map size, the number, locations and size of obstacles')
    print('[lx ly lz n x_ob1 y_ob1 z_ob1 lx_ob1 ly_ob1 lz_ob1 x_ob2 y_ob2 z_ob2 lx_ob2...]')
    graph_s = raw_input()
    graph = list(map(float, graph_s.split()))
    assert len(graph)%6 == 4
    print('Processing...')
    
    waypoints = list()
    running_time = list()

    Astar(init_goal,str(graph_s))
    Model3D(init_goal, waypoints)

