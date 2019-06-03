import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys


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
        
        # Start_Goal_point information [3*1,3*1]
          
        sentence = str(Start_Goal_point)
        self.data = list(map(float, sentence.split()))
        self.openset = [] # (node_total_cost,node)
        self.closedset = []
        self.optimal_path = []
        
        self.info = str(graph)
        
        self.map_Generate()   
        self.Astar_run()
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
        fig = plt.figure()
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
        goal_position = np.array([self.data[3],self.data[4],self.data[5]])
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
                print(np.array(self.optimal_path))
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
        plt.show()
            
            
if __name__ == '__main__':
    
    print('')
    print('Please set the initial and goal configurations [q0,qh] = [x0 y0 z0 xh yh zh]')
    ini_goal_s = input()
    ini_goal = list(map(int, ini_goal_s.split()))
    if not len(ini_goal) == 6:
        print('WRONG input, exiting...')
        sys.exit()  
    print('Please set the map size, the number, locations and size of obstacles')
    print('[lx ly lz n x_ob1 y_ob1 z_ob1 lx_ob1 ly_ob1 lz_ob1 x_ob2 y_ob2 z_ob2 lx_ob2...]')
    graph_s = input()
    graph = list(map(int, graph_s.split()))
    if not len(graph)%6 == 4:
        print('WRONG input, exiting...')
        sys.exit()     
    print('Processing...')
    
    Astar(str(ini_goal_s),str(graph_s))

