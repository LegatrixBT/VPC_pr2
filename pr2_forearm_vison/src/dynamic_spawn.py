#!/usr/bin/python2.7

 '''
 File: dynamic_spawn.py
 Project: Stage UFPE Recife, BR, 2019-2020
 File Created: Tuesday, 21st April 2020 12:07:52 pm
 -----
 University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 -----
 Last Modified: Tuesday, 21st April 2020 12:08:37 pm
 Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 -----
 Copyright a choisir - 2020 Etudiant
 -----
 HISTORY:
 Date       	By	Comments
 -----------	---	---------------------------------------------------------
 '''





import rospy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import GetWorldPropertiesResponse
from gazebo_msgs.msg import *
from geometry_msgs.msg import Pose


class Properties_gazebo:
    """Contain the Gazebo gathered info"""

    def __init__(self):

        self.models = {}
        self.proxy_world_properties = []
        self.properties = ''

        self.__get_properties__()

    def __get_properties__(self):
        """ Get all the properties and states models """
        self.models = {}
        self.proxy_world_properties = []
        self.properties = ''

        try:
                
            rospy.wait_for_service('/gazebo/get_world_properties')
            self.proxy_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            self.str = str(self.proxy_world_properties())

            # get the names and properties of models in dictionnary
            for model in self.proxy_world_properties().model_names:
                try:
                    proxy_model_info = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    self.models[model] = proxy_model_info(model, '')

                except rospy.ServiceException as e:
                    rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        except rospy.ServiceException as e:
            rospy.loginfo("Get World Properties service call failed:  {0}".format(e))

    def __repr__(self):

        return str(self.str)

    def get_model_state(self,name):
        pass

    def insert_sdf_object(self, path, object_name, posx=1, posy=1, posz=1, rx=0, ry=0, rz=0, rw=1):
        """Insert an sdf object if no other with the same name exist in the gazebo world"""

        rospy.init_node('insert_object', log_level=rospy.INFO)

        initial_pose = Pose()
        initial_pose.position.x = posx
        initial_pose.position.y = posy
        initial_pose.position.z = posz

        initial_pose.orientation.x = rx
        initial_pose.orientation.y = ry
        initial_pose.orientation.z = rz    
        initial_pose.orientation.w = rw    


        f = open(path,'r')
        sdff = f.read()

        if not object_name in properties_world.models:
            try:
                rospy.wait_for_service('/gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(object_name, sdff, "robotos_name_space", initial_pose, "world")
                rospy.loginfo("Spawn Sdf Model successfuly called : " + str(object_name) + " spawnned in world")
                # update the properties
                self.__get_properties__()

            except rospy.ServiceException as e:
                rospy.loginfo("Spawn Sdf Model service call failed: {0}".format(e))
        else:
            rospy.loginfo("A model with the same name already exist")

    



properties_world = Properties_gazebo()
properties_world.insert_sdf_object('/usr/share/gazebo-9/models/VS_landmark/model.sdf', 'VS_landmark', 0.83056, 0.14666, 1.25, 0.0, -0.45)
print properties_world  
#print properties_world.models['VS_landmark']
print properties_world.models

## call du service pour stoper le projecteur de texture 
"""
rosservice call /camera_synchronizer_node/set_parameters "config:
  bools:
  - {name: '', value: false}
  ints:
  - {name: 'projector_mode', value: 1}
  strs:
  - {name: '', value: ''}
  doubles:
  - {name: '', value: 0.0}
  - {name: '', state: false, id: 0, parent: 0}" 
"""

                #### ajouter les entree au dico de prop puis
                # petit menu pour choisir 'afficher le statut de qui on veut 
                # fin du script 









