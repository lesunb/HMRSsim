import pytest
import os
import sys

import xml.etree.ElementTree as ET
import simulator.utils.helpers as helpers
# Include current directory in Python path
sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.getcwd(), "simulator"))

from collision import Vector
from simulator.resources.load_resources import inflate
from simulator.models.Wall.Wall import Wall
from simulator.components.Collidable import Collidable
from simulator.components.Position import Position


@pytest.fixture
def parse_map():
  test_map = """<mxfile host="Electron" modified="2020-07-02T23:13:04.237Z" agent="5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) draw.io/13.3.1 Chrome/83.0.4103.119 Electron/9.0.5 Safari/537.36" etag="3MINz4I9WtMW7lj2hSMH" version="13.3.1" type="device"><diagram id="np8vLNK_3szPsoOlPMPO" name="Wall render test">1ZbBbqMwEIafhmOrGEPSHBuSTbTaSqut1Fa9VA4YsGoYZJxC+vRrlyGETav0QHeXXGL/M7bH/2cDDg2yeq1Ykd5AxKXjTqLaoUvHdcnE9cyfVfaN4s/dRkiUiDCpE27FK29HoroTES97iRpAalH0xRDynIe6pzGloOqnxSD7qxYs4SfCbcjkqXovIp3iLq5op2+4SFJc2SMY2LLwOVGwy3G5HHLeRDLWzoJbLFMWQXUk0ZVDAwWgm1ZWB1xaV1vDmnHfPogeKlY8158Z8P2eLB+4voPosdLkial97F0gthcmd+gEFqv3rTUvXGlhnPrBtlz+hFJoAbkJbUFryBy6SHUmTZ+YZpt7LUViczQURmXYC02lXBnBWFHY2bM6scfpMpYAqpAsv6yYqZkuYiFlABLUWwl0Yn92/tM9ow12YV4fSejBmkPGtdqbFIxeIQ48qLTtVx12b4ZaeoS8PakMT1pymLnz3DTQ9vcRvMplOJ0FajNPfvHoZr56XpcXZDwIjK2LSChzAZv1S9hZz4bA4vt9LuQUC3mHCp19FRZ3PFimH2BRoBlK5tkwCCYy72M6PN3OcCKTr+JEx8PJP8/J84fh5E7/4ET/NacRvWnoeU7UsxcqlqLYYEEDQPPo/3a5/PFA8z4BjQx0u+hfvF2m230jvsWOPsHp6jc=</diagram></mxfile>"""
  root = ET.fromstring(test_map)
  map_frame = ET.fromstring(inflate(root[0].text, True))
  map_content = map_frame[0]
  assert map_frame.tag == 'mxGraphModel'
  assert map_content.tag == 'root'
  return (map_frame, map_content)

@pytest.fixture
def get_walls(parse_map):
  frame, content = parse_map
  WIDTH = int(frame.attrib.get('pageWidth', 500))
  HEIGHT = int(frame.attrib.get('pageHeight', 500))
  walls = []
  for cell in content:
    if cell.tag == 'mxCell' and cell.attrib.get('style', None) is not None:
      cell_style = helpers.parse_style(cell.attrib['style'])
      shape = cell_style.get('shape', '')
      # This test is only about Walls
      if shape == 'mxgraph.floorplan.wall':
        walls.append(Wall.from_mxCell(cell, (WIDTH, HEIGHT)))
  # There are 6 walls in the map
  assert len(walls) == 6
  return walls

def test_wall_type(get_walls):
  for w in get_walls:
    assert type(w) == Wall

def test_position(get_walls):
  for w in get_walls:
    color = w.draw.style['fillColor']
    if color == '#000001':
      # Big horizontal wall
      assert w.pos.x == 80
      assert w.pos.y == 23
      assert w.pos.w == 470
      assert w.pos.h == 10
      assert w.pos.angle == 0
    elif color == '#000002':
      # Big vertical wall
      assert w.pos.x == 550
      assert w.pos.y == 33
    elif color == '#000003':
      assert w.pos.x == 430
      assert w.pos.y == 93
      assert w.pos.angle == 340
    elif color == '#000004':
      assert w.pos.x == 360
      assert w.pos.y == 83
      assert w.pos.angle == 315
    elif color == '#000005':
      assert w.pos.x == 260
      assert w.pos.y == 83
      assert w.pos.angle == 45
    elif color == '#000006':
      assert w.pos.x == 190
      assert w.pos.y == 93
      assert w.pos.angle == 20

def test_collision(get_walls):
  for w in get_walls:
    col_points = w.boundaries.points
    pos_points = list(map(lambda x: Vector(x[0], x[1]), w.pos._get_box()))
    assert col_points == pos_points

def test_render(get_walls):
  for w in get_walls:
    render_points = w.draw._get_points()
    pos_points = w.pos._get_box()
    assert list(pos_points) == list(render_points)
    





