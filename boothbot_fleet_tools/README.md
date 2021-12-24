# Instructions for Running the Simulator
Last updated on: Dec. 22, 2021.

## 1. Setting Up

### Docker
1. Go to `docker/boothbot-fleet-docker` directory.
1. Build docker image using

   `./build_fleet_manager.sh`

### Change compose file settings
1. Replacing `{{ var }}` inside `docker-compose-fleet.yml` to your own path.

### Run
1. Go to `docker` directory
2. Run containers using

    `docker-compose -f ./docker-compose-fleet.yml up`

### Login
1. Using following command to attach

    `docker exec -it fleet_manager bash`
