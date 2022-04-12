# stereo-sew
Stereographic SEW (shoulder-elbow-wrist) angle for 7-DOF robot arms

# Running the Sawyer Simulation in WSL with Windows 10

## Install WSL2 and Ubunutu 18.04
Windows Subsystem for Linux (WSL) lets you run linux on your Windows machine.

https://docs.microsoft.com/en-us/windows/wsl/install

## Install ROS Melodic and the Sawyer simulation
https://sdk.rethinkrobotics.com/intera/Gazebo_Tutorial


## Install XMing on Windows
XMing is an X11 server to display linux the GUIs on Windows. The X11 server runs on Windows and handles displaying GUIs, while the programs on Linux (like Gazebo) are X11 clients. If you're using Windows 11 instead of Windows 10, you may not need to use XMing as WSL handles GUIs for you.

https://sourceforge.net/projects/xming/

Run XLaunch from XMing with the following options
- [x] Multiple Windows
- Display number -1
- [x] Start no client
- [ ] Native opengl
- [x] Disable access control

Before running any program with a GUI, run
```
export GAZEBO_IP=127.0.0.1
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
export LIBGL_ALWAYS_INDIRECT=0
```
(You can add this to `.bashrc` or `intera.sh` so it gets run automatically)

Running `gazebo` should open up a Gazebo window.

## Run the simulation in WSL
In `intera.sh`, make the change
```your_ip = $(hostname -I | tr -d ' ')```

The `tr -d ' '` part removes extra whitespace

(Hardcoding the WSL IP address doesn't work because it changes every time you start WSL.)

Now if you run 
```
./intera.sh sim
roslaunch sawyer_gazebo sawyer_world.launch
```
you should see a Gazebo window with Sawyer.

# Controlling Sawyer from MATLAB with ROS
https://sdk.rethinkrobotics.com/intera/Arm_Joints#Joint_Control_Mode

## Install Visual Studio Community 2019 C++ Compiler
This is for  for compliling custom ROS messages in MATLAB. Depending on your version of MATLAB you may need to install the 2017 version instead, or upgrade MATLAB
Don't forget to run `mex -setup cpp` in MATLAB.

https://www.mathworks.com/support/requirements/supported-compilers.html

https://www.mathworks.com/support/requirements/previous-releases.html

https://visualstudio.microsoft.com/vs/older-downloads/

## Install python 3
As documented in the links below, the version of python from the windows store won't work.
I also had to run `pyenv('Version','3.9')` in MATLAB.

https://www.mathworks.com/content/dam/mathworks/mathworks-dot-com/support/sysreq/files/python-compatibility.pdf

https://www.mathworks.com/help/matlab/matlab_external/install-supported-python-implementation.html

## Compile custom messages
In MATLAB, run `rosgenmsg('./')` in one folder up from `intera_core_msgs`. In particular, we need to compile `JointCommand.msg`.

https://github.com/RethinkRobotics/intera_common/tree/master/intera_core_msgs

I had to to temporarily turn off antivirus to get the compiler to work

https://techsupportwhale.com/mt-exe-general-error-c101008d-failed-to-write-the-updated-manifest-to-the-resource-of-file/

To get connect to the ROS network in WSL, you can use
```
[status,ip_address] = system("wsl hostname -I")
ip_address = strtrim(ip_address)
rosinit(ip_address)
```

Now you can send a joint message like the following, which sawyer should respond to
```
header:
  seq: 1938
  stamp:
    secs: 34
    nsecs: 658000000
  frame_id: ''
mode: 1
names: [right_j6, right_j5, right_j4, right_j3, right_j2, right_j1, right_j0]
position: [0.44306910945301803, 1.817598298595035, 2.7958235468001615, -1.589300730783936, 0.010672899352333655, -3.1244247205300306, 2.3654556749206463]
velocity: []
acceleration: []
effort: []
```
