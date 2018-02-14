FROM robobenklein/home:latest

RUN sudo apt-get -y update
RUN sudo apt-get -y install \
      cmake g++ 

# deps for rip build
RUN sudo apt-get -y install \
      libssh2-1-dev \
      lcov \
      libssl-dev

COPY --chown=901:901 . /home/${LUSER}/rip


