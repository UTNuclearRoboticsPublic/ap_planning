# AP Planning
Code for pre-planning screw-based AP moves

# Install
```sh
sudo apt install python3-vcstool
git clone -b <BRANCH> https://github.com/UTNuclearRobotics/ap_planning.git # fill in proper branch name
vcs import < ap_planning/repos.yaml
rosdep install --from-paths . --ignore-src -y
catkin build ap_planning
```
