ARG IDF_VERSION=release-v5.4

# Stage 1: base ESP-IDF
FROM espressif/idf:${IDF_VERSION} AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Vienna

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=${TZ}

RUN apt-get update && apt-get install -y --no-install-recommends \
      locales sudo bash-completion vim clang-format picocom \
      dbus bluez bluetooth libbluetooth-dev usbutils \
    && sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && locale-gen en_US.UTF-8 \
    && ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies used by micro-ROS build
COPY requirements.txt /tmp/requirements.txt
RUN /bin/bash -c "source /opt/esp/idf/export.sh && pip install --no-cache-dir -r /tmp/requirements.txt" \
    && rm /tmp/requirements.txt

# Stage 2: dev (non-root user)
FROM base AS dev

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN if id -u ${USER_UID} >/dev/null 2>&1; then \
      userdel --force $(id -un ${USER_UID}) 2>/dev/null || true; \
    fi

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && usermod -aG dialout,plugdev ${USERNAME}

COPY prompt.sh /opt/devcontainer/prompt.sh
COPY entrypoint.sh /opt/devcontainer/entrypoint.sh
RUN chmod +x /opt/devcontainer/entrypoint.sh /opt/devcontainer/prompt.sh

RUN echo 'export IDF_EXPORT_QUIET=1' >> /home/${USERNAME}/.bashrc \
    && echo 'source /opt/esp/idf/export.sh > /dev/null 2>&1' >> /home/${USERNAME}/.bashrc \
    && echo 'source /opt/devcontainer/prompt.sh' >> /home/${USERNAME}/.bashrc

RUN mkdir -p /workspaces/firmware && chown -R ${USERNAME}:${USERNAME} /workspaces

USER ${USERNAME}
WORKDIR /workspaces/firmware

ENTRYPOINT ["/opt/devcontainer/entrypoint.sh"]
CMD ["/bin/bash"]
