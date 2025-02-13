# LAYERED exercise

You are an engineer at LAYERED, everyone is happy and the sun is shining, when suddenly Davide arrives at your desk. He is super excited and speaks loudly, he found a ROS package online and he guarantees you that it would be a super cool package to add to the current stack. Unfortunately, only a ROS1 version exists. He is currently working on a super urgent thing and he saw you playing chess at your desk so he was wondering if you could help him migrate this package from ROS1 to ROS2. He also tells you that he didn't have much time to look at the code in details so it might be badly written.

At LAYERED, we are strong advocates for excellence in our work. To us, *excellence* means:

- **Team-first mindset:** Work as a team for the team. Ensuring everything you do is understandable and align with team guidelines. Remember, major successes and failures are often the sum of many individual contributions.
- **Work smart:** Always look for ways to simplify your work. Build efficient processes, automate repetitive tasks, and document best practices. Always balance effort and impact, ensuring your time is spent where it matters most.
- **Continuous improvement:** Enhance what can be improved, document what should be improved. Always question existing processes, apply creativity and leadership to drive progress—while recognizing that everything can’t change at once. Meaningful change are the result of small incremental changes.
- **Commitment to growth:** Always strive for your best, never settle for less, and seek guidance when needed. Have confidence in your abilities but stay humble enough to learn and improve.

## Launch Instructions

Instructions to run the package using ROS1:

1.  Enable other processes to access the X11 server: 
    ```
    xhost local:root
    ``` 
2. Download and start the ROS noetic docker image: 
    ```
    docker run -it --rm  --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw  osrf/ros:noetic-desktop-full
    ``` 
3. Install required dependencies: 
    ```
    sudo apt update && sudo apt install git-all python3-catkin-tools -y
    ```
4. Create a catkin workspace and build the exercise: 
    ```
   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && git clone https://github.com/LAYERED-pierrechass/layered_exercise.git && cd .. && catkin build && source devel/setup.bash
   ```
5. Launch the exercise:  
   ```
   roslaunch layered_exercise turtle.launch
   ```

> **DISCLAIMER:** This exercise was developed and tested using Ubuntu 24.04.1. It is recommended to use the same OS to make sure everything runs smoothly. However, because it runs inside a docker you should be able to run it on any platform by slightly adapting some of the instructions above. You can also use a docker, a dual boot or a VM to run Ubuntu 24.04.1. 

## Help
If you need help for anything or have any question, please feel free to contact either Davide or Pierre. Asking for help will never be devalued. 
- Pierre: pchassagne@ethz.ch
- Davide: dbarret@ethz.ch

## Deadline
If you are currently reading this it means that you have two days to complete the exercise and send us your submission.

## Submission
Once you are done, you can send us, in the format you want (zip, github repos, ...), all the documents necessary to run your exercise. Note that we are using Ubuntu 24.04 and you can assume that we don't have anything installed on our machine.

# LAYERED Coding Conventions

## General

In general we are trying to follow the ROS2 coding style and conventions.

As specified in the [Google Coding Style guide](https://google.github.io/styleguide/cppguide.html) names should be optimized for readability using names that would be clear even to people on a different team.

We use a longer `line length` of 120.

### Package naming

All package should start with the prefix `rosp_`, which initially meant robotic on-site plastering, but is now kept for posterity.

## C++

ROS2 follows the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). We do as well.

### Exceptions

**Function naming**: We use `mixedCase` for function naming. 

## Python

ROS2 follows the [PEP8 Style Guide](https://peps.python.org/pep-0008/). We do as well.

### Exceptions

**Function naming**: We use `mixedCase` for function naming.

## Code documentation

Documentation regarding details of implementation and repos should live inside the README.md of the repos. Code documentation should cover every non trivial technical aspects of the code as well as the workflow to use it. Make it as concise as possible, use diagrams and make it visual.