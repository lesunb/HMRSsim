import click
import json
import re
import os
from simulator.typehints.build_types import ConfigParseError

from simulator.utils.validators import validate_config


@click.group()
@click.version_option(package_name='HMRsim_lesunb')
def main():
    pass

def parse_json_option(ctx, param, value):
    if value is None: 
        return None
    value = re.sub(r"'", r'"', value)
    try:
        return json.loads(value)
    except json.JSONDecodeError as err:
        raise click.BadParameter(f'{err}')


@click.command()
@click.option('--file', '-f',
    default='simulation.json',
    type=click.Path(exists=True),
    help='Path to config file. MUST be json.',
    show_default=True)
@click.option('--json', '-j',
    type=click.UNPROCESSED,
    callback=parse_json_option,
    help='JSON-like config object.')
def configtest(file, json):
    """Tests a config object for HMRsim simulation."""
    config = json if json is not None else file
    try:
        resp = validate_config(config)
    except ConfigParseError as err:
        click.echo('Analysis aborted:')
        click.echo(err)
        return
    if len(resp) == 0:
        click.echo('Config OK ✔')
        return
    click.echo(f'{len(resp)} errors found in config:')
    click.echo('\n- '.join(resp))

@click.command()
@click.option('--path', default='.', type=click.Path(), show_default=True)
def create_project(path):
    """Creates the defaul folder tree structure for a HMRsim project"""
    os.makedirs(path, exist_ok=True)
    os.chdir(path)
    os.mkdir('components')
    os.mkdir('builders')
    os.mkdir('models')
    os.mkdir('systems')
    open('simulation.json', 'w').close()
    open('run.py', 'w').close()
    click.echo(f'Created HMRsim project structure. Root is {os.path.abspath(path)}')



main.add_command(configtest)
main.add_command(create_project)
