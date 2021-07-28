import logging
from datetime import datetime, timedelta
from simulator.typehints.dict_types import SystemArgs

from queue import Queue
traces = []


def process(kwargs: SystemArgs):
    env = kwargs['ENV']
    event_store = kwargs['EVENT_STORE']
    sleep = env.timeout
    total = timedelta()
    traces.append('env_time,total_time,avg_simulation_second,event_store_size\n')
    while True:
        start = datetime.now()
        yield sleep(1)
        end = datetime.now()
        total += end - start
        traces.append(f'{env.now},{total},{total / env.now},{len(event_store.items)}\n')


def clean():
    logger = logging.getLogger(__name__)
    with open('traces.csv', 'w') as fd:
        logger.debug(f'There are {len(traces)} trace messages to include in file')
        fd.writelines(traces)
