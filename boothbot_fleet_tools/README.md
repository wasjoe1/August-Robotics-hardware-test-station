This scripts are mainly for development use, as them are already
integrated into our CI/CD repository.

How to Use?
----
0. Go to `docker` directory;
1. Set environment in `.env`;
2. Build docker image: `docker build -t boothbot_fleet .`;
3. Build source file: `docker-compose -f docker-compose.build.ym up`
4. Run fleet server: `docker-compose up`

First Time User Creation
----
For security, the user must be created by logging in the fleet_main container,
and run django's createsuperuser subcommand.