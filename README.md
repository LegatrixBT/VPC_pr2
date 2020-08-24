# VPC_pr2
Implémentation d'asservissement visuel prédictif sur PR2

Pour lancer l'asservissement visuel, dans l'ordre : 
 1. roslaunch gazebo_perso pr2_target_world.launch gui:=false
 2. rosrun pr2_forearm_vison landmark_detection_pr2.py 
 3. rosrun vpc_simple_controller main_vpc 
 
Note : il est possible que le porjecteur de texture ne soit pas bien éteint. Dans ce cas tuer Gazebo (1) et le relancer. 
Lancer ensuite le détecteur de qrCode (2). L'asservissement peut être démarré via (3).
