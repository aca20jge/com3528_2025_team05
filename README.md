# com3528team05


this repository contains a script to have miros engage in social/play
behaviour. it contains a script, play_behaviour.py which 

you must save the repository to your catkin_ws/src directory in your mdk 
environment. then, you must build the team05_assignment package using 
'catkin build'.


once the package has been built, there are two options for running the 
simulation:

1. roslaunch team05_assignment play_behaviour.launch
   this will run the script in a world containing seven miro agents.
   leaving the script unedited, the launch file should run fine and
   you will see the seven agents begin to socialise.
   
2. rosrun team05_assignment play_behaviour.py
   for this command to work, you will need to have an environment
   containing one or more miros already open. this can be achieved
   by following these steps:
   
   a) run the "miro_sim" command, which launches a simulation environment
      with one miro.

   b) remove the blue ball from the environment (click on it and press the
      delete key).

   c) add more miros to the environment by navigating to "Insert" (menu in
      top left) and clicking "MiRo Robot". add as many miros as you like.

   d) [important] in play_behaviour.py you must change self.N_MIROS to the
      number of miros you have placed in the simulation.

   e) finally, you can run the script using

      rosrun team05_assignment play_behaviour.py
