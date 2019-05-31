
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# In[2]:


# Function to plot a cube
def plot_linear_cube(ax, x, y, z, dx, dy, dz, color):
    xx = [x-dx/2, x-dx/2, x+dx/2, x+dx/2, x-dx/2]
    yy = [y-dy/2, y+dy/2, y+dy/2, y-dy/2, y-dy/2]
    kwargs = {'alpha': 1, 'color': color}
    ax.plot3D(xx, yy, [z-dz/2]*5, **kwargs)
    ax.plot3D(xx, yy, [z+dz/2]*5, **kwargs)
    ax.plot3D([x-dx/2, x-dx/2], [y-dy/2, y-dy/2], [z-dz/2, z+dz/2], **kwargs)
    ax.plot3D([x-dx/2, x-dx/2], [y+dy/2, y+dy/2], [z-dz/2, z+dz/2], **kwargs)
    ax.plot3D([x+dx/2, x+dx/2], [y+dy/2, y+dy/2], [z-dz/2, z+dz/2], **kwargs)
    ax.plot3D([x+dx/2, x+dx/2], [y-dy/2, y-dy/2], [z-dz/2, z+dz/2], **kwargs)


# In[3]:


# Unfinished function for generate the map
class Grid_map:
    def __init__(self,graph):
        # Add ros here to get map information [map_size, #_obstacles, location_of_obstacles, size_of_obstacles]
        #[3*1,1,6*1,6*1...]
        # To test the code, I use 'graph' to represent the information get from user

        
        # Size of quadrotor
        quadrotor_x = 0.92
        quadrotor_y = 0.92
        quadrotor_z = 0.29
        extend_xy = quadrotor_x**(0.5)
        extend_z = (quadrotor_x**2+quadrotor_z**2)**(0.5)
        
        # Generate obstacle expansion graph
        fig = plt.figure()
        ax = Axes3D(fig)         
        info = str(graph)
        map_info = list(map(int, info.split()))
        map_size = np.array([map_info[0],map_info[1],map_info[2]])
        plot_linear_cube(ax, map_size[0]/2, map_size[1]/2, map_size[2]/2, map_size[0], map_size[1], map_size[2], 'black')
        obs_num = map_info[3]
        obs_size = np.zeros((3))
        obs_loc = np.zeros((3))
       
        for i in range(obs_num): 
            obs_loc = np.array([map_info[6*i+4],map_info[6*i+5],map_info[6*i+6]])
            obs_size = np.array([map_info[6*i+7],map_info[6*i+8],map_info[6*i+9]])  
            obs_extend = np.array([map_info[6*i+7]+extend_xy,map_info[6*i+8]+extend_xy,map_info[6*i+9]+extend_z])
            plot_linear_cube(ax, obs_loc[0], obs_loc[1], obs_loc[2], obs_size[0], obs_size[1], obs_size[2], 'red')
            plot_linear_cube(ax, obs_loc[0], obs_loc[1], obs_loc[2], obs_extend[0], obs_extend[1], obs_extend[2],'blue')
        plt.show()


# In[4]:


def takeFirst(elem):
    return elem[0]


# In[5]:


class Node:
    def __init__(self,p_cost,h_cost,t_cost,posi,ind):
        self.past_cost = p_cost
        self.heur_cost = h_cost
        self.total_cost = t_cost
        self.parent = [] # Elements of that list should be node, which will be set in Grid_map
        self.neighbor = [] # Elements of that list should be (node,weight)
        self.position = np.array(posi)
        self.index = ind


# In[6]:


class Astar:
    def __init__(self, Start_Goal_point):
        
        # Add ros here to get Start_Goal_point information [3*1,3*1]
        # rospy.init_node('Astar')
        # rospy.Subscriber("Astar", String, self.callback)
        # rospy.spin()
    
        # Grid_map(graph)
        self.s_point = Node(0,20,20,[0,0,0],1)       
        self.g_point = Node(10000,0,10000,[6,6,6],6)        
        self.node2 = Node(10000,10,10010,[2,2,2],2)
        self.node3 = Node(10000,10,10010,[3,3,3],3)
        self.node4 = Node(10000,10,10010,[4,4,4],4)
        self.node5 = Node(10000,10,10010,[5,5,5],5)
       
        self.node2.neighbor = [(self.node3,27),(self.g_point,10)]
        self.node3.neighbor = [(self.s_point,18),(self.node2,27),(self.g_point,15)]
        self.node4.neighbor = [(self.s_point,12),(self.node5,8),(self.g_point,20)]
        self.node5.neighbor = [(self.s_point,30),(self.node4,8),(self.g_point,10)]
        self.g_point.neighbor = [(self.node2,10),(self.node3,15),(self.node4,20),(self.node5,10)]
        self.s_point.neighbor = [(self.node3,18),(self.node4,12),(self.node5,30)]
        
        sentence = str(Start_Goal_point)
        data = list(map(float, sentence.split()))
        self.s_point.position = np.array([data[0],data[1],data[2]])
        self.g_point.position = np.array([data[3],data[4],data[5]])
        self.openset = [] # (node_total_cost,node)
        self.openset.append((self.s_point.total_cost,self.s_point))
        self.closedset = []
        self.optimal_path = []        
        self.Astar_run()

 
    def Astar_run(self):
        
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
                    
if __name__ == '__main__':
    Astar(str('0 0 0 6 6 6'))

