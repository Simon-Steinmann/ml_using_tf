import rospy
import numpy
from math import pi, sqrt
import gym
from gym import spaces
from gym.envs.registration import register
from sensor_msgs.msg import JointState
from joint_pos_2_tf import Pos2Tf  


    # Register Env-----------------------------------------------------------------------------------------------
register(
        id='j2n6s300Test-v3',
        entry_point='task_env_tf:j2n6s300TestEnv',

    )



class j2n6s300TestEnv(gym.Env):
    def __init__(self):
        
        # Variable for the training script--------------------------------------------------------------------
        self.action_space = spaces.Discrete(rospy.get_param("/j2n6s300/n_actions"))

        # Variables for the Env--------------------------------------------------------------------------------
        self.action = 0
        self.init_positions = [0.0, 2.9, 1.3, -2.07, 1.4, 0.0, 0, 0, 0, 0, 0, 0.0] 
        self.target_point = [0.5,0.5,0.5]
        self.joint_pos_increment_value = rospy.get_param("/j2n6s300/pos_increment")
        self.n_step = 0
        self.n_episode = 0 
        self.info = 'running'
        self.joint_names =  ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5',
              'j2n6s300_joint_6', 'j2n6s300_joint_finger_1', 'j2n6s300_joint_finger_tip_1', 'j2n6s300_joint_finger_2',
              'j2n6s300_joint_finger_tip_2', 'j2n6s300_joint_finger_3', 'j2n6s300_joint_finger_tip_3']    
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.joint_state = JointState()
        self.joint_state.name = self.joint_names
        rospy.on_shutdown(self.shutdown_hook)   
        self.Pos2Tf = Pos2Tf()
        self.reset()
        rospy.logwarn('Environment ready!')
        
#------------------------------------------------------------------------------------------------------------------
#----------------Gym Env Methods------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        

    def step(self, action):
        #print action
        joints = self.joint_state
        positions = joints.position
        action_pos = list(self.init_positions)        
        for i in xrange(6):
            action_pos[i] = positions[i]
            if action == 0:
                action_pos[i] = positions[i] - self.joint_pos_increment_value
            elif action == 1:
                action_pos[i] = positions[i] + self.joint_pos_increment_value
            action -= 2
        #self.joint_state_pub(action_pos)

        self.n_step += 1  
        obs = self.get_obs(action_pos)
        info = self.info        
        reward = self.get_reward(obs)
        reward = 0
        done = self.is_done()           
        return obs, reward, done, info
        

    def reset(self):
        self.joint_state_pub(self.init_positions)
        self.n_step = 0
        obs = self.get_obs(self.init_positions)
        return obs


    def close(self):
        """
        Function executed when closing the environment.
        Use it for closing GUIS and other systems that need closing.
        :return:
        """
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")
        

#------------------------------------------------------------------------------------------------------------------
#----------------Support functions needed for the mein gym functions------------------------
#------------------------------------------------------------------------------------------------------------------
     
    def get_obs(self, positions):
        obs =  numpy.zeros(48)
        tf_msg = self.Pos2Tf.calculate_joints(positions, 6)
        for  i in xrange(6):
            tran = tf_msg[i].transform.translation
            rot = tf_msg[i].transform.rotation
            obs[i*8+0] = round(tran.x,3)
            obs[i*8+1] = round(tran.y,3)
            obs[i*8+2] = round(tran.z,3)
            obs[i*8+3] = round(rot.x,3)
            obs[i*8+4] = round(rot.y,3)
            obs[i*8+5] = round(rot.z,3)
            obs[i*8+6] = round(rot.w,3)
            obs[i*8+7] = round(positions[i],3)
            
            
            
        return(obs)   
        
        
    def get_reward(self, obs):
        deltax = obs[5*3+0]-self.target_point[0]
        deltay = obs[5*3+1]-self.target_point[1]
        deltaz = obs[5*3+2]-self.target_point[2]
        distance = sqrt(deltax**2+deltay**2+deltaz**2)
        
        if distance < 0.3:
            reward = 1
        else:
            reward = 0
        reward = min(500,1/(distance**2))
        self.reward = reward
        return reward
        
    def is_done(self):
        if self.n_step > 999 or self.reward  > 200:
            return True
        else:            
            return False
            


    
    def joint_state_pub(self,positions ):  
        #update joint_state
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.position = positions
                
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        
        while not rospy.is_shutdown():
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(self.joint_state)
                break
          """          
          
          
    def shutdown_hook(self):
        rospy.logwarn('Env shutdown')
        self.info = 'user abort'