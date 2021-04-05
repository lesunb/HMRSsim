import logging
from datetime import datetime, timedelta
from typehints.dict_types import SystemArgs

from queue import Queue
traces = []


def process(kwargs: SystemArgs):
    env = kwargs['ENV']
    sleep = env.timeout
    total = timedelta()
    runs = 0
    traces.append('runs,total,env_time,avg_second')
    while True:
        start = datetime.now()
        yield sleep(1)
        end = datetime.now()
        total += end - start
        runs += 1
        traces.append(f'{runs},{total},{env.now},{total / runs}\n')


def clean():
    logger = logging.getLogger(__name__)
    with open('traces.csv', 'w') as fd:
        logger.debug(f'There are {len(traces)} trace messages to include in file')
        fd.writelines(traces)
