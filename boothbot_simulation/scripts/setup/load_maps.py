#!/usr/bin/python
from backoffice_conf import setup
from maps.map_transfer import Import
import os

MAPS_PATH = "../../maps"

def load_maps():
    maps = os.listdir(MAPS_PATH)
    for map in maps:
        Import(os.path.join(MAPS_PATH, map))

if __name__ == "__main__":
    setup()
    
    load_maps()