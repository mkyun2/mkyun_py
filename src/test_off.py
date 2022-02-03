#! /usr/bin/env python3
from threading import local
import rospy
import sys
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Imu
#from sensor_msgs.msg import mag
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandHome, SetMode
from mavros_msgs.msg import HomePosition
from mavros_msgs.msg import State
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from tf.transformations import euler_from_quaternion
import pcl
import pcl.pcl_visualization
import numpy as np
import ros_numpy
import math
import signal
from datetime import datetime
#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
#fig= plt.figure()
#angle_v=np.cos(np.dot(np.arange(0,256,1),0.3515625))
#print(math.cos(np.arange(0,255,1)*np.array(0.3515625)))
#des_ext_angle = np.array([])mavros/local_postion/pose_cov
#depth_map=np.zeros((12,12))

frames=[]

init_yaw=0
yaw=0
rotation_mat = np.zeros((3,3))
rotation_mat_lidar = np.zeros((3,3))
pseu_rot_lidar = np.zeros((3,3))
desired_position = np.zeros((3,1))#NEU
#desired_position = np.array([[0.0],[15.0],[2.0]])#ENU
local_position = np.zeros((3,1))
ranges = np.zeros((121))
F=np.zeros((3,1))
R=np.zeros((3,1))
    #local_position = PoseStamped()

def state_callback(data):
    global current_state
    current_state = data

def imu_callback(data):
    global init_yaw
    global yaw
    global rotation_mat
    global rotation_mat_lidar
    global pseu_rot_lidar
    global desired_position
    des_pos = np.array([[15.0],[1.0],[2.0]])#ENU
    quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    #print('yaw',yaw*180.0/np.math.pi)
    #print(yaw)
    #if (yaw<0.0):
    #    yaw=yaw+2.0*np.math.pi
    rotation_mat_lidar = np.array([[np.math.cos(yaw),-np.math.sin(yaw),0],
                             [np.math.sin(yaw),np.math.cos(yaw),0],
                             [0                ,0                ,1]])
    
    #pseu_rot_lidar = np.linalg.pinv(rotation_mat_lidar)
    #print(des_pos.shape)
    #print(rotation_mat.shape)
    
    #desired_position[:2]=des_pos[:2]
    #print(desired_position)
    #if current_state.mode == "STABILIZED":
    
    if current_state.mode == "AUTO.LAND":
        init_yaw=yaw
        rotation_mat = np.array([[np.math.cos(init_yaw),-np.math.sin(init_yaw),0],
                                [np.math.sin(init_yaw),np.math.cos(init_yaw),0],
                                [0                ,0                ,1]])
        #pseu_rot = np.transpose(rotation_mat)
        #desired_position[:3]=@des_pos
        desired_position[:3]=np.matmul(rotation_mat,des_pos)+local_position
        #print('1',desired_position)
        #desired_position[:3]=np.matmul(pseu_rot,des_pos)+local_position
        #print('2',desired_position)
        #print('yaw',init_yaw*180.0/np.math.pi,yaw*180.0/np.math.pi)
def local_pose_callback(data):
    global local_position
    local_position[0]=data.pose.position.x#E N
    local_position[1]=data.pose.position.y#N W
    local_position[2]=data.pose.position.z#U U
    
def lidar_callback(data):
    point_map_x = []
    point_map_y = []    
    

    for p in pc2.read_points(data,skip_nans=True,field_names=("x","y","z")):
        point_angle = np.math.atan2(p[0],-p[1])*180.0/np.math.pi
        #print(p[0],p[1],p[2])
        if(point_angle>30.0 and point_angle<150.0 and abs(p[2])<0.06):#and abs(p[2])<0.05
            #print(p[0],p[1],p[2])
            #print(round(point_angle))
            ranges[round(abs(point_angle-30))] = np.math.sqrt((np.math.pow(p[0],2)+np.math.pow(p[1],2)))
            #point_map_x.append(int(p[0]*1000))
            #point_map_y.append(int(p[1]*1000))   
    #print(len(ranges),ranges)
    #frames.append((point_map_y,point_map_x))int_angle<150.0 and abs(p[2])<0.06):#and abs(p[2])<0.05
            #print(p[0],p[1],p[2])
            #print(round(point_angle))

            
            #point_map_x.append(int(p[0]*1000))
            #point_map_y.append(int(p[1]*1000))   
    #print(len(ranges),ranges)
    #frames.append((point_map_y,point_map_x))p_x.append(int(p[0]*1000))
            #point_map_y.append(int(p[1]*1000))   
    #print(len(ranges),ranges)
    #frames.append((point_map_y,point_map_x))
    
            #print(depth_map.shape)
            #point_map_y.append(p[1])
    
    #print('------------------------------------------------')
    #print('------------------------------------------------')
    #print('------------------------------------------------')
    
    #pc = ros_numpy.numpify(data)
    #print(pc.shape)
    #points=np.zeros((pc.shape[0],3))
    
    #print(ranges)
    #print(angle_v.shape,pc[15][0:256])
    #print('x: ', np.dot(pc[15][0:256],angle_v))#0.3515625 1개 당 deg #x좌표계에 대해서만
    #plt.imshow(pc[15][:256])
    #plt.plot()
    #p = pcl.PointCloud(np.array(points, dtype=np.float32))
def Velocity_Pub(des_position,local_position,lidar_range):
    cmd_vel = Twist()

    distance = np.linalg.norm((des_position[:2]-local_position[:2]))
    #print(distance)
    K_att = 0.5
    K_rep = 0.1
    #F_rep_x=0.0
    #F_rep_y=0.0
    #local_position=local_position
#외란이 있을법한 확률 장애물이 있을법한
    pos_error_E = np.squeeze(des_position[0]-local_position[0])#E N
    pos_error_N = np.squeeze(des_position[1]-local_position[1])#N W 
    pos_error_U = np.squeeze(des_position[2]-local_position[2])#U U
    des_yaw = np.math.atan2(pos_error_N,pos_error_E)
    #if(des_yaw<0.0):
    #        des_yaw=des_yaw+2*np.math.pi
    yaw_error = des_yaw - yaw
    global F
    global R
    global ranges
    F=np.zeros((3,1))
    R=np.zeros((3,1))
    #print(pos_error_N, distance)
    F[0]=K_att*pos_error_E/distance#E
    F[1]=K_att*pos_error_N/distance#N
    F[2]=K_att*pos_error_U
    #print('des',des_position,'local',local_position)
    #print(pos_error_N,pos_error_E,pos_error_U)
    ####추후 numpy이용해서 한번에 연산하기 연산속도++
    for n in range(0,120):
        #print(n,lidar_range[n])
        obs_pos_N = lidar_range[n]*np.math.sin((n+30)*np.math.pi/180.0)#N
        obs_pos_E = lidar_range[n]*np.math.cos((n+30)*np.math.pi/180.0)#E
        #print(n,lidar_range[n],obs_pos_x,obs_pos_y)
        
        obs_distance = lidar_range[n]
        
        if(obs_distance<6.0 and obs_distance>0.5):
            R[0]=R[0]+(K_rep*(1/obs_distance - 1/6.0))/np.math.pow(obs_distance,2)*(obs_pos_E/obs_distance)
            R[1]=R[1]+(K_rep*(1/obs_distance - 1/6.0))/np.math.pow(obs_distance,2)*(obs_pos_N/obs_distance)
            
        else:
            R[0]=R[0]#E
            R[1]=R[1]#N
    #print(R.shape,rotation_mat.shape)
    R=rotation_mat_lidar@R
    #print(rotation_mat)
    #print(R)
    #if (abs(yaw_error)<0.3):
    if (abs(pos_error_U)<0.2):
        cmd_vel.linear.x = F[0] - R[0]#E
        cmd_vel.linear.y = F[1] - R[1]#N
    if(abs(pos_error_N)<0.2 and abs(pos_error_E)<0.2):
        cmd_vel.linear.y = 0
        cmd_vel.linear.x = 0
    #print(lidar_range)
    #print(np.math.atan2(desired_position[1],desired_position[0]),yaw)
    #print('yaw:error:',yaw_error)
    cmd_vel.angular.z=0.2*yaw_error
    cmd_vel.linear.z=F[2]
    #print(F[0],F[1],R[0],R[1])
    local_vel_pub.publish(cmd_vel)
    #print(ranges)
    ranges=np.zeros((121))
#def func(each_frame):
    #plt.clf()
    #x,y = each_frame
    #plt.scatter(x,y)
    
def main():
#publish node init
    
    
    #10hz frequency
    rate = rospy.Rate(20) 
    count=0

    f= open('/home/mkyun/log1.txt','w')
    #set_home(True,yaw,)
    while not rospy.is_shutdown():
        #ani = FuncAnimation(fig,func,frames=frames)
        #plt.show()
        #Velocity_Pub(desired_position,local_position,ranges)
        
        #if(des_yaw<0.0):
        #    des_yaw=des_yaw+2*np.math.pi
        #print(ranges)
        if (count%30)==0:
                
            f.write(datetime.now().strftime('-%d %H:%M:%S')+'\n')
            f.write('des_pos:'+str(desired_position)+'loc_pos:'+str(local_position)
            +'\n')
            f.write('des_yaw:'+str(R[0])+'yaw:'+str(R[1])
            +'\n')
        #print(F,R)
        
        if(count>100):
            #print(desired_position)
            Velocity_Pub(desired_position,local_position,ranges)
        #print(desired_position)
        #print('yaw:',yaw*180.0/np.math.pi,'des_yaw:',des_yaw*180.0/np.math.pi)
        #ranges=np.zeros((120))
        rate.sleep()
        #ranges=np.zeros((120))
        count+=1
    f.close()
    rospy.spin()
if __name__=='__main__':
    try:
    #init
        rospy.init_node('test_off',anonymous=False)
    #publisher name
        #set_home = rospy.ServiceProxy('/mavros/cmd/set_home',CommandHome)
        state_sub = rospy.Subscriber('mavros/state',State,state_callback)
        lidar_sub = rospy.Subscriber('/os_cloud_node/points',PointCloud2,lidar_callback)
        imu_sub = rospy.Subscriber('mavros/imu/data',Imu,imu_callback)
        pose_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,local_pose_callback)
        local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=20)
        main()
    except rospy.ROSInterruptException:
        pass
