
-------------------------------------------------------------------------------------------
Bashrc
-------------------------------------------------------------------------------------------
```
alias eb='gedit ~/.bashrc'  
alias sb='source ~/.bashrc'  
alias cm='cd ~/catkin_ws && catkin_make'  
alias arm='rosservice call /mavros/cmd/arming 1'  
alias disarm='rosservice call /mavros/cmd/arming 0'  
alias qgc='~/Downloads/qgc/./QGroundControl.AppImage'  
alias cmonly='cd ~/catkin_ws && catkin_make --only-pkg-with-deps'  
alias csb='source ~/cartographer_ws/install_isolated/setup.bash'  
alias ccm='cd ~/cartographer_ws && catkin_make_isolated --install --use-ninja'  
```
