#!/bin/bash

# Allow X11 connections from Docker
xhost +local:docker

# Build and run the simulation container
docker compose -f docker-compose.sim.yml up --build

# Clean up X11 permissions on exit
xhost -local:docker