# Initial setup

<!-- TODO: Note in overview of packages used and sources -->

```
mkdir -p ~/cme_ws/src
cd ~/cme_ws/src
catkin_init_workspace
git clone https://github.com/MyNameIsCosmo/kkim_cme
./kkim_cme/bootstrap_ws.sh
```

If bootstrap.sh fails:
```
sudo apt install python3-pip ros-kinetic-moveit ros-kinetic-joint-state-publisher-gui ros-kinetic-teleop-twist-keyboard
/usr/bin/python3.5 -m pip install --user vcstool

vcs import --recursive <~/cme_ws/src/kkim_cme/.vcsinstall

find ~/cme_ws/src/kinova-ros -maxdepth 1 -type d -not -ipath '*/.*' -exec touch {}/CATKIN_IGNORE \;
find ~/cme_Ws/src/ros_kortex -maxdepth 1 -type d -not -ipath '*/.*' -exec touch {}/CATKIN_IGNORE \;

cd ~/cme_ws
rosdep update -y
rosdep install --from-paths src --ignore-src --skip-keys="kinova_description kortex_driver" -y
catkin_make -j$(nproc)
. ~/cme_ws/devel/setup.bash
```

# Dependencies
```
cd ~/cme_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Setup Catkin
The CME packages will pull in the ROS Husky dependencies.

*NOTE*: I use `cme_ws` here instead of `catkin_ws` since I do not want mixed dependencies
```
mkdir -p ~/cme_ws/src
cd ~/cme_ws/src
catkin_init_workspace
git clone git+ssh://git@github.com/MyNameIsCosmo/kkim_cme.git
```

## Pull dependencies
[VCSTool] is used to pull dependencies automatically at a given version.  
To view the dependencies pulled, check out the [.vcsinstall](.vcsinstall) file.
```
sudo apt install python3-vcstool
cd ~/cme_ws/src
vcs import . <kkim_cme/.vcsinstall
```

## Setup Kinova Software
<!-- TODO (Cosmo): conan may not provide reproducible builds...
	opt for their "old build method":
	https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/examples/readme.md#not-using-conan -->  
```
sudo apt install python3 python3-pip
python3 -m pip install --user conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default

cd ~/cme_ws/src
git clone https://github.com/Kinovarobotics/ros_kortex.git
```

## Pull dependencies and make software

```
cd ~/cme_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
catkin_make
```

[VCSTool]: https://github.com/dirk-thomas/vcstool
