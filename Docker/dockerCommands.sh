docker build -t drone-connect .
docker run -it --rm --privileged --device=/dev/ttyUSB0 --device=/dev/ttyUSB0 --device=/dev/ttyAMA0 --env-file .env --name drone-debug drone-connect
docker exec -it <container_id> bash

docker exec -it <container_id_or_name> bash

docker stop $(docker ps -q)

docker rm -f $(docker ps -aq)

docker image prune -f

docker logs -f <container_id_or_name>

docker ps -a

docker images

ls /dev/ttyUSB*
