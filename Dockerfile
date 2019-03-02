FROM ros:melodic

COPY . /robot
WORKDIR /robot

RUN bash -c "source /opt/ros/melodic/setup.bash && \
             rosdep install --from-paths src --ignore-src -y && \
             catkin_make && \
             echo \"source /robot/devel/setup.bash\" >> ~/.bashrc"
