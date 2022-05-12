FROM robotgit.localdom.net:5000/ros2/eloquent-onbuild

LABEL version="0.1.1"
LABEL ros.distro=${ROS_DISTRO}
LABEL vcs-url="http://robotgit.localdom.net/ai-box/components/sensopart/sensorpart_connector"
LABEL maintainer="Rasmus Lunding Henriksen <rlh@teknologisk.dk>"
LABEL description="This image will automatically start a sensopart_connector node."

ENV PACKAGE_NAME="sensopart_connector"
ENV LAUNCHFILE_NAME="sensopart_connector.launch.py"
