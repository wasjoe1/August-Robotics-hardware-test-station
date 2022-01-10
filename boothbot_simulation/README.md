# Instructions for Running the Simulator

Last updated on: Dec. 30, 2021. by Patrick

## Summary

This package is for running the boothbot project under simulation environment, so
that developers could test their code at their own PC.

## Dependency

1. docker
2. docker-compose
3. Boothbot project docker image

## Usage

1. cd `boothbot_simulation/docker`
2. edit `setup.bash` with your own local environments
3. source setup.bash
4. **IMPORTANT** use `dc_gen_docker_mirror_dir` to generate necessary folders
5. dc_build, kill&exit after finished.
6. dc up

It's running. Use `dc_debug` to attach on `roscore` container for debugging
