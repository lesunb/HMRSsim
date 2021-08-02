import os
import pytest
from simulator.typehints.build_types import ConfigParseError
import simulator.utils.validators as validators

working_dir = os.path.dirname(os.path.realpath(__file__))
data_dir = os.path.join(working_dir, '..', 'data')

def test_validate_entity_definition():
    entity_definition = {}
    resp = validators.validate_entity_definition(entity_definition)
    # Missing 3 required entries, should return 3 errors
    assert len(resp) == 3
    entity_definition['entId'] = 'id'
    entity_definition['isObject'] = True
    entity_definition['isInteractive'] = True
    resp = validators.validate_entity_definition(entity_definition)
    # All required fields are present and with correct type, should return no errors
    assert len(resp) == 0

def test_validate_config():
    config = {}
    resp = validators.validate_config(config)
    assert len(resp) == 0
    config['context'] = './test'
    resp = validators.validate_config(config)
    assert len(resp) == 0
    with pytest.raises(ConfigParseError):
        validators.validate_config('missing_file.txt')
