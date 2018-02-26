FROM utkrobotics/rip_deps:latest
# $LUSER is user with 901:901 from home image

# copy in RIP source code
COPY . /home/${LUSER}/code/rip
RUN sudo chown -R 901:901 /home/${LUSER}/code/rip

WORKDIR /home/${LUSER}/code/rip
