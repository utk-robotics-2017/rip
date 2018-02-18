FROM utkrobotics/rip_deps:latest
# $LUSER is user with 901:901 from home image

# copy in RIP source code
COPY --chown=901:901 . /home/${LUSER}/code/rip

WORKDIR /home/${LUSER}/code/rip
