from python:3

ARG USER_ID
ARG GROUP_ID

# Don't attempt to set the user to the root user (uid=0) or group (gid=0)
RUN if [ ${USER_ID:-0} -eq 0 ] || [ ${GROUP_ID:-0} -eq 0 ]; then \
        groupadd mfrc522 \
        && useradd -g mfrc522 mfrc522 \
        ;\
    else \
        groupadd -g ${GROUP_ID} mfrc522 \
        && useradd -l -u ${USER_ID} -g mfrc522 mfrc522 \
        ;\
    fi \
    && install -d -m 0755 -o mfrc522 -g mfrc522 /home/mfrc522 \
    && mkdir -p /etc/sudoers.d  \
    && echo "mfrc522 ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/mfrc522-all-nopasswd

RUN apt-get update && apt-get -y install --no-install-recommends \
    vim \
    sudo \
    less \
    && rm -rf /var/lib/apt/lists/*

COPY ./docker-files/home/.* /home/mfrc522/

COPY ./requirements.txt /tmp

RUN pip install -r /tmp/requirements.txt

USER mfrc522

WORKDIR /mfrc522-reader
