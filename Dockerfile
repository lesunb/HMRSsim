FROM ros:galactic-ros-core

RUN apt-get update && apt-get install -y curl build-essential python-is-python3 ros-galactic-rosbridge-server
RUN curl -sSL https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py

COPY src/ /etc/hmrsim/src
COPY ./pyproject.toml /etc/hmrsim/pyproject.toml
COPY ./setup.cfg /etc/hmrsim/setup.cfg
COPY ./LICENSE /etc/hmrsim/LICENSE
COPY ./README.md /etc/hmrsim/README.md

WORKDIR /etc/hmrsim
RUN pip install .

WORKDIR /usr/app

COPY ./rosbridge_entrypoint.sh /rosbridge_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/rosbridge_entrypoint.sh"]
