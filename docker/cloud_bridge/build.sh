#!/bin/bash

echo -n "Enter the username of the repo > "
read repouser
echo "Enter the password of the repo > "
read repopass

TAG=${DOCKER_IMAGE_TAG:-"kinetic"}
docker build --no-cache --build-arg REPO_USER=$repouser --build-arg REPO_PASS=$repopass -t="localhost:5000/robopaas/rr_bridge:${TAG}" .
