FROM python:3.9.5


COPY ./Pipfile ./Pipfile
COPY ./Pipfile.lock ./Pipfile.lock
RUN pip install pipenv
RUN pipenv install --system

COPY src/ /etc/hmrsim/src
COPY ./pyproject.toml /etc/hmrsim/pyproject.toml
COPY ./setup.cfg /etc/hmrsim/setup.cfg
COPY ./LICENSE /etc/hmrsim/LICENSE

WORKDIR /etc/hmrsim
RUN pip install -e .

WORKDIR /usr/app