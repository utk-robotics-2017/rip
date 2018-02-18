FROM robobenklein/home:latest
# $LUSER is user with 901:901 from home image

RUN sudo apt-get -y update
RUN sudo apt-get -y install \
      cmake g++ 

# deps for rip build
RUN sudo apt-get -y install \
      libssh2-1-dev \
      lcov \
      libssl-dev \
      libqt5opengl5-dev \
      libeigen3-dev \
      libsuitesparse-dev

# Install g2o
COPY ./g2o /home/${LUSER}/code/g2o
RUN sudo chown -R ${LUSER}:${LUSER} /home/${LUSER}

WORKDIR /home/${LUSER}/code/g2o
RUN mkdir build
WORKDIR /home/${LUSER}/code/g2o/build
RUN cmake ..
RUN make -j$(nprocs --ignore=1 )
RUN sudo make install

#COPY --chown=901:901 . /home/${LUSER}/code/rip

#WORKDIR /home/${LUSER}/code/rip
WORKDIR /home/${LUSER}
