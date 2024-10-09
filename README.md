# CRAZYSIM - Docker

[This guide is used to create this](https://github.com/Los-UruBots-del-Norte/CrazySim)

### Requirements
- Ubuntu 20 or 22
- Docker
- Docker-compose 1.29.2 - [Follow this guide](https://medium.com/@jared.ratner2/setting-up-docker-and-docker-compose-with-nvidia-gpu-support-on-linux-716db95c0f7c)
- NVIDA Drivers to run with gpu - Non necessary

### X11 
To see Gazebo, run this command on a terminal first:

```bash
xhost +
```

### Env file
Create a .env or rename the .env-example file with this content:

```bash
DISPLAY=host.docker.internal:0
```

### How to Run Docker-compose without GPU
```bash
docker compose up
```

### How to Run Docker-compose with GPU
```bash
docker compose -f docker-compose-gpu.yml up
```

### Access to the container
Execute this commands in order on a new terminal:

```bash
docker ps
```
Copy the id example ***a6c31d880b65*** and execute:

```bash
docker exec -it a6c31d880b65 /bin/bash
```

### Simulation on Docker

This simulation is running when the container is created.