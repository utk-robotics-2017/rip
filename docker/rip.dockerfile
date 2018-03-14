FROM utkrobotics/rip_deps:latest
# $RUSER is user with 902:902 from rip_deps

# copy in RIP source code
COPY . /home/${RUSER}/code/rip
RUN sudo chown -R 902:902 /home/${RUSER}

WORKDIR /home/${RUSER}/code/rip

# vim: set syntax=dockerfile:
