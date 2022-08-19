FROM ros:foxy-ros-core

RUN apt-get update && apt-get install -y curl build-essential python-is-python3 ros-foxy-rosbridge-server ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*'
RUN curl -sSL https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip install "poetry==1.1.13"

COPY src/ /etc/hmrsim/src
COPY examples/ /etc/hmrsim/examples
COPY ./pyproject.toml /etc/hmrsim/pyproject.toml
COPY ./poetry.lock /etc/hmrsim/poetry.lock
COPY ./LICENSE /etc/hmrsim/LICENSE
COPY ./README.md /etc/hmrsim/README.md

WORKDIR /etc/hmrsim
RUN poetry install

COPY ./hmrsim_entrypoint.sh /hmrsim_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/hmrsim_entrypoint.sh"]
