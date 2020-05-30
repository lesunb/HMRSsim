import pytest
import os
import xml.etree.ElementTree as ET

from simulator.utils.mxgraph import inflate

working_dir = os.path.dirname(os.path.realpath(__file__))
data_dir = os.path.join(working_dir, 'data')

def test_inflate_drawio_xml():
    #content = readFromCompresedXml(data_dir+'/room.drawio')
    tree = ET.parse(data_dir + '/room.drawio')
    root = tree.getroot()
    for child in root:
        decoded_content = inflate(child.text, b64=True)
        diagram = ET.fromstring(decoded_content)
        print(diagram)
    content = "ok"
