#!/bin/bash
xhost +local:root
docker run --rm \
  --entrypoint /bin/bash \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -it brass/cp2
xhost -local:root
