# Macro-Polo Robots (MPBots): Pursuer-Evader Simulation

The reference papers for this repository are listed below:  
[Background: Motion and Odometry](https://github.com/dhipatel21/Pursuer-Evader/blob/11a27ab67caf4a3ae0a0fdc6bb4d2b1ebab4591a/Botlab_Part_1.pdf)  
[Background: SLAM](https://github.com/dhipatel21/Pursuer-Evader/blob/11a27ab67caf4a3ae0a0fdc6bb4d2b1ebab4591a/Botlab_Part_2.pdf)  
[Background: Implementing Planning and Exploration](https://github.com/dhipatel21/Pursuer-Evader/blob/11a27ab67caf4a3ae0a0fdc6bb4d2b1ebab4591a/Botlab_Part_3.pdf)  
[Pursuit-Evasion](https://github.com/dhipatel21/Pursuer-Evader/blob/11a27ab67caf4a3ae0a0fdc6bb4d2b1ebab4591a/robotics-report.pdf)
## Background
The Pursuit-Evasion (PE) Problem has long attracted the interest of the robotics and autonomy field. Such algorithms typically, but not always, assume perfect information - namely, that both predator and prey know the others location, and the map environment. More robust solutions have addressed sensing limitations present in the real world, whether it be a limited sensor range or knowledge of only a bearing. It is this variant of the classic PE problem which we propose to explore and implement.  
The bearing-only PE problem is characterized by two main challenges. First; that neither pursuer nor evader are aware of the other’s position. Second; that guaranteed target convergence is reliant on a continuous bearing. We propose to introduce two additional constraints: first, that impeding obstacles lie on the straight-line bearing from pursuer to evader; and second, that the bearing of the target may be discontinuous (caused by, for example, an occluding obstacle). These additional constraints will necessitate either the implementation of new pursuit logic, or of new control logic. We plan to pursue the latter strategy, and determine whether the guarantees of the family of pursuit logic presented in hold under these non-ideal control circumstances. We then plan to explore more contemporary solutions which take obstacles in the arena into account in the behavioral planning stage.
Our project is essentially broken into three components, which will be pursued in parallel.
1) Obscured bearing pursuit behavior.  
2) Auditory bearing determination, and visual contingency preparation.  
3) High-speed path planning and control.  
These efforts have produces two MPBots able to conduct our modi- fied pursuit-evasion game effectively. The MPBots are able to, alternately, detect sound or emit sound, and give chase or attempt evasion. We have expanded the functionalities of the MBot platform, and have also upgraded their underlying control abilities. Furthermore; we have developed a visual navigation contingency, using a MPBot- mounted wide-angle camera and April Tags as an alternate method of calculating and supplying the relative bearing from pursuer to evader to Marco. This has eliminated a major source of risk stemming from our reliance on an experimental method of bearing generation, which is susceptible to ambient noise levels.  
Our project concludes with a series of tests and demonstrations of the two robots in various arenas. These arenas will be enclosed (to prevent robot escape or human intrusion), and will be littered with obstacles to provide meaningful navigational challenges to Marco and Polo. Utilizing these obstacles, Marco and Polo will begin in line- of-sight of each other, to provide an initial bearing when in visual pursuit mode, or in any position and heading while in audio pursuit mode.

## Basics of The Simulation
At first, the positions of the MPBots are unknown to one another. After a sound pulse, or in the visual case, target acquisition, the pursuit-evasion game begins. The respective autonomy modules send commands to Marco and Polo, which are interpreted by the motion planner to ensure no environmental collisions, and to plan the intermediate states using the A* search algorithm. Finally, once Marco comes into range of Polo, a shutdown signal is broadcast, and both MPBots cease operation.

## Conclusion
This project set out to extend the sensing, control, and autonomy functionalities of the MPBot platform to enable the performance of a pursuit-evasion game. Despite technical difficulties with the UMA-8 directional microphone, processing limitations which restricted the MPBot to the use of a single tracking camera, and general platform nonlinearities (unicylce dynamics are one of the worst dynamical models for both pursuit and evasion, due to its reliance on heading for movement), we have largely succeeded. We first demonstrated the ability to track a moving April Tag target, plan around obstacles, reacquire the target after a loss of lock, and successfully intercept the evader. We then demonstrated the ability to replicate this behavior using audio-only tracking with the UMA-8 directional microphone array, proving the original concept. The upgraded MPBot firmware, designed with modularity in mind, will be able to be used in future projects, attempting more ambitious autonomous behaviors.

##
#### === Directories ===

= bin/
    - where all built binaries are located
    - you'll be using this directory a lot
    
= data/
    - where data needed to run parts of the assignment are located
    - log files and target files for SLAM and exploration are here
    
= lcmtypes/
    - where the .lcm type definitions are located
    - the generated types are stored in src/lcmtypes
    
= lib/
    - where static libraries are saved during the build process
    - you should never need to manually do anything in this directory
    
= src/
    - where all source code for botlab is located
    - the subdirectories will have a further description of their contents
    

#### === Files ===

= Makefile
    - the root Makefile that launches the recursive build of the botlab code
    - you shouldn't need to edit this file
    
= log_mbot_sensors.sh
    - a script to log the sensor data needed for SLAM so you can easily create your own log files 
      for testing

= setenv.sh
    - a script for setting environment variables needed for running Vx applications
    - run this script before running botgui in a terminal on your laptop
    - run via `. setenv.sh` -- note the space

### BREAK IN CASE OF GIT CORRUPTION
// Backup your git repo \\
find .git/objects/ -type f -empty | xargs rm
git fetch -p
git fsck --full
