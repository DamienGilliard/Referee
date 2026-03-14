# Referee
Referee is a georeferenced point cloud registration method that limits the error propagation of registration errors in the georeferencing of the point cloud

> [!CAUTION]
> This registration method is currently in development and the main branch is not yet fully functional.


## Build instructions for ubuntu:

A full installation guide will be available once the main branch is functional, but in the mean time we can state that this code is developped on ubuntu 24.04 and macos sequoia, and that the [INSTALL.md](INSTALL.md) currently contains simplified installation instructions that should work fine, but are subject to change.

## Container for cross-platform reproducibility

A docker image is also provided in order to run this cone on any machine. To Run this code, you will need Docker installed on your computer. Instructions to install Docker can be found [here](https://docs.docker.com/get-started/introduction/get-docker-desktop/). Once Docker installed, you will need to open a terminal and go to Referee's root directory:

```bash
cd <path-to>/Referee
```
Once you are there, build the docker image by running: 
```bash
# For Unix/Linux (including macOS):
sudo docker build -t referee .

# For Windows (using Command Prompt or PowerShell, with admin privilege if necessary):
docker build -t referee .
```
the `-t` means "tag", and the argument that follows is the tag we will give to the container (in our case `referee`) and the `.` means the Dockerfile is located in the cuirrent directory

This step can take a long time (about 20 min on my computer)

Then run the code thanks to this command:
```bash
# For Unix/Linux (including macOS):
sudo docker run -it -v $(pwd)/Results:/home/Referee/src/build/output referee

# For Windows (using Command Prompt or PowerShell, with admin privilege if necessary):
docker run -it -v %cd%\Results:/home/Referee/src/build/output referee
```

`-v $(pwd)/Results:/home/Referee/src/build/output` means that the `/home/Referee/src/build/output` folder in the container should be mounted in the `Result` folder of the "bare-metal" computer.

## Result
The goal of the registration algorithm is obviously to have a point cloud, but in line with the assignment's goal the output of the Docker container is the `One_page_synthesis.md` in the `./Results` folder. A PDF of the same document is provided as reference of the result I get.