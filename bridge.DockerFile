FROM ros:foxy-ros-core

RUN apt-get update && apt-get install -y build-essential python-is-python3 ros-foxy-rosbridge-server
RUN apt-get -y dist-upgrade

COPY ./bridge_entrypoint.sh /bridge_entrypoint.sh

ENTRYPOINT ["/bin/bash", "/bridge_entrypoint.sh"]