import json
import pathlib

from typing import Dict, List, Union
from simulator.typehints.build_types import ConfigParseError
from simulator.typehints.dict_types import LogLevel

VALIDATE_TYPES = [
    ('context', [str]),
    ('map', [str]),
    ('FPS', [int, float]),
    ('DLW', [int, float]),
    ('duration', [int, float]),
    ('verbose', [LogLevel, int]),
    ('verbose', [LogLevel, int]),
    ('simulationComponents', [dict]),
    ('extraEntities', [list]),
    ('simulatorConfigOptions', [dict])
]

def validate_config(config: Union[str, Dict]) -> List[str]:
    """ Validates a config object, passed as a dict or a string to json file"""
    if isinstance(config, str):
        path = pathlib.Path(config)
        if not path.exists():
            raise ConfigParseError(f'File {path.absolute()} not found')
        if path.suffix != '.json':
            raise ConfigParseError(f'File {path.absolute()} is not a JSON file. Only JSON is supported.')
        with open(path, 'r') as fd:
            config = json.loads(fd.read())
    # Parse config
    errors_found = []
    # Type checking
    for key, expected_types in VALIDATE_TYPES:
        value = config.get(key, None)
        if value is not None and not (type(value) in expected_types):
            errors_found.append(f'{key} should be {expected_types}. {type(value)} found.')
    # Check extra types
    if 'simulationComponents' in config:
        simulator_components = config['simulationComponents']
        for key, value in simulator_components.items():
            if not isinstance(value, list):
                errors_found.append(f'You should pass a list to initialize simulation component {key}. {type(value)} found.')
    if 'extraEntities' in config:
        errors_found += validate_entity_definition(config['extraEntities'])
    if 'map' in config:
        map_file = pathlib.Path(config.get('context', '.')) / config['map']
        if not map_file.exists():
            errors_found.append(f'Map file {map_file.absolute()} not found')
    return errors_found


def validate_entity_definition(definition: Dict) -> List[str]:
    """ Validates an entityDefinition object"""
    errors = []
    types = [
        ('entId', str),
        ('components', dict),
        ('isObject', bool),
        ('isInteractive', bool),
        ('name', str),
        ('type', str)
    ]
    for key, expected_type in types:
        value = definition.get(key, None)
        if value is not None and not isinstance(value, expected_type):
            errors.append(f'{key} should be {expected_type}. {type(value)} found.')
    required = [('entId', int), ('isObject', bool), ('isInteractive', bool)]
    for key, required_type in required:
        if not (key in definition):
            errors.append(f'{key} (type {required_type}) is required for entityDefiniton')
    return errors
