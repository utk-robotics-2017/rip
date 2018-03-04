FROM robobenklein/home:latest
# $LUSER is user with 901:901 from home image

# deps for rip build
RUN sudo install_clean \
      cmake g++ git curl \
      xz-utils fakeroot automake \
      doxygen \
      libssh2-1-dev \
      lcov \
      libssl-dev \
      libqt5opengl5-dev \
      libeigen3-dev \
      libsuitesparse-dev

# Create a new robot user:
USER root
ENV RUSER=dogcow
RUN groupadd -r ${RUSER} -g 902
RUN useradd -m -u 902 -r -g 902 ${RUSER}
RUN adduser ${RUSER} sudo
USER ${RUSER}
WORKDIR /home/${RUSER}
RUN bash ~robo/code/configs/provision.sh

# Install g2o
COPY external/g2o /home/${RUSER}/code/g2o
RUN sudo chown -R ${RUSER}:${RUSER} /home/${RUSER}

WORKDIR /home/${RUSER}/code/g2o
RUN mkdir build
WORKDIR /home/${RUSER}/code/g2o/build
RUN cmake ..
RUN make -j$(nproc --ignore=1 )
RUN sudo make install

