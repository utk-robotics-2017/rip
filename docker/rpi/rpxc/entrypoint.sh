#!/bin/zsh

# This is the entrypoint script for the dockerfile. Executed in the
# container at runtime with root privileges.

# /dev should be mounted as a tmpfs by docker like
# --tmpfs=${RPXC_SYSROOT}/dev:rw,dev
chroot "$SYSROOT" "$QEMU_PATH" /bin/bash -c '\
 mknod -m 622 /dev/console c 5 1 \
 && mknod -m 666 /dev/null c 1 3 \
 && mknod -m 666 /dev/zero c 1 5 \
 && mknod -m 666 /dev/ptmx c 5 2 \
 && mknod -m 666 /dev/tty c 5 0 \
 && mknod -m 444 /dev/random c 1 8 \
 && mknod -m 444 /dev/urandom c 1 9 \
 && chown root:tty /dev/{console,ptmx,tty} \
'

# If we are running docker natively, we want to create a user in the container
# with the same UID and GID as the user on the host machine, so that any files
# created are owned by that user. Without this they are all owned by root.
# If we are running from boot2docker, this is not necessary, and you end up not
# being able to write to the volume.
# The rpxc script sets the RPXC_UID and RPXC_GID vars.
if [[ -n $RPXC_UID ]] && [[ -n $RPXC_GID ]]; then

    RPXC_USER=rpxc-user
    RPXC_GROUP=rpxc-group
    RPXC_HOME=/home/$RPXC_USER

    groupadd -o -g $RPXC_GID $RPXC_GROUP 2> /dev/null
    useradd -o -m -d $RPXC_HOME -g $RPXC_GID -u $RPXC_UID $RPXC_USER 2> /dev/null

    # Run the command as the specified user/group.
    HOME=$RPXC_HOME exec chpst -u :$RPXC_UID:$RPXC_GID -- $@
else
    # Just run the command as root.
    exec $@
fi
