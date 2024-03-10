#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import json
import pygame
import requests

url = "http://127.0.0.1:9030/jsonrpc"
cmd = {
    "method": "SetBusServoPulse",
    "params": [],
    "jsonrpc": "2.0",
    "id": 0,
}

step_width = 10

# Map keyboard keys to servo actions
key_map = {
    pygame.K_LEFT: {"axis": 0, "increment": -step_width},
    pygame.K_RIGHT: {"axis": 0, "increment": step_width},
    pygame.K_UP: {"axis": 1, "increment": -step_width},
    pygame.K_DOWN: {"axis": 1, "increment": step_width},
    pygame.K_a: {"axis": 2, "increment": -step_width},
    pygame.K_d: {"axis": 2, "increment": step_width},
    pygame.K_w: {"axis": 3, "increment": -step_width},
    pygame.K_s: {"axis": 3, "increment": step_width},
}

connected = False
change = [500, 500, 136, 931, 795, 500]

pygame.init()
pygame.display.set_mode((100, 100))  # Create a window for keyboard events

while True:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key in key_map:
                axis = key_map[event.key]["axis"]
                increment = key_map[event.key]["increment"]
                change[axis] += increment
                change[axis] = max(0, min(1000, change[axis]))
                cmd["params"] = [20, 1, axis + 1, change[axis]]
                r = requests.post(url, json=cmd).json()

    time.sleep(0.06)
