# Fluxus-10.0-IIT-Indore

- ## Problem Statement for Task 1 :    
  Move the end effector of an 8 joint Modular snake robot to any arbitrary point in 3D space, whose tail is fixed at origin and alternate joints can rotate along Z and Y axis (revolute joints with 1 DOF)

![Fluxus-10.0-IIT-Indore](/images/model.png)

- ## Requirements/Installations :    
  1. Ubuntu 20.04 LTS Desktop version.
  2. Python3 (3.7+).
  3. git. If not installed, type following commands on terminal   
        ``` 
        sudo apt-get update
        ```
        ```
        sudo apt-get install git
        ```
  4. mujoco, numpy libraries. Type following commands on terminal to install these libraries
        ```
        pip3 install mujoco
        ```
        ```
        pip3 install numpy
        ```
   
  5. Mujoco - Physics simulating engine.    
    - Download Mujoco 2.3.6 from [here](https://github.com/deepmind/mujoco/releases/download/2.3.6/mujoco-2.3.6-linux-x86_64.tar.gz)    
    - Extract the zip file and copy the mujoco-2.3.6 folder to home directory.
  
    - Open terminal and type the following command to install `libglfw` package.  
    
        ``` 
        sudo apt install libglfw3-dev
        ```   
    - Navigate to `sample` directory inside `mujoco-2.3.6` in terminal and type :
  
        ```
        make
        ```
    - Open terminal and type : 
       
      ```
      sudo -H nautilus
      ```

      and the press `ctrl + L` and write `/usr/lib` in the address bar and press enter.    
    - Now copy the contents inside `mujoco-2.3.6/lib` folder and paste them to current(/usr/lib) folder. 
---

- ## Running the simulation :
  
![Fluxus-10.0-IIT-Indore](/images/model_code.png)    

  1. Clone this repo in *mujoco-2.3.6* directory : 
   
        ```
        git clone https://github.com/ab31mohit/Fluxus-10.0-IIT-Indore.git 
        ``` 
  2. now go to *Fluxus-10.0-IIT-Indore*   
  3. run the working_code.py file :   
      ``` 
      python3 working_code.py
        ```


