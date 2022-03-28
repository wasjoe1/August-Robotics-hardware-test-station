How to Use?
----
0. Go to `docker` directory;
1. Set environment in `.env`;
2. Build docker image: `docker build -t boothbot_fleet .`;
3. Build source file: `docker-compose -f docker-compose.build.ym up`
4. Run fleet server: `docker-compose up`