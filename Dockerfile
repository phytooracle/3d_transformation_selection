FROM ubuntu:18.04

WORKDIR /opt
COPY . /opt

USER root
ARG DEBIAN_FRONTEND=noninteractive
ARG PYTHON_VERSION=3.7.1
ENV IRODS_USER=phytooracle
ENV IRODS_PASSWORD=mac_scanalyzer

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    gnupg2 \
    build-essential \
    software-properties-common \
    apt-utils \
    libgl1-mesa-glx \
    ffmpeg \
    libsm6 \
    libxext6 \
    libffi-dev \
    libbz2-dev \
    zlib1g-dev \
    libreadline-gplv2-dev \
    libncursesw5-dev \
    libssl-dev \
    libsqlite3-dev \
    sqlite3 \
    tk-dev \
    libgdbm-dev \
    libc6-dev \
    liblzma-dev \
    libxrender-dev \
    libgl1-mesa-dev \
    python3-dev \
    locales

# Install PROJ 6.3.1
RUN wget https://download.osgeo.org/proj/proj-6.3.1.tar.gz && \
    tar -xvzf proj-6.3.1.tar.gz && \
    cd proj-6.3.1 && \
    ./configure && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd .. && \
    rm -rf proj-6.3.1 proj-6.3.1.tar.gz

# Install PROJ datumgrid data files using find + xargs
RUN wget https://download.osgeo.org/proj/proj-datumgrid-1.8.tar.gz && \
    tar -xvzf proj-datumgrid-1.8.tar.gz && \
    mkdir -p /usr/local/share/proj && \
    find proj-datumgrid-1.8 -type f \( -name "*.gsb" -o -name "*.gtx" -o -name "*.dat" \) | xargs -I {} cp {} /usr/local/share/proj/ && \
    rm -rf proj-datumgrid-1.8 proj-datumgrid-1.8.tar.gz

# Download and build Python
RUN cd /opt && \
    wget https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tgz && \
    tar xzf Python-${PYTHON_VERSION}.tgz && \
    cd Python-${PYTHON_VERSION} && \
    ./configure --with-ensurepip=install && \
    make install && \
    cd .. && \
    rm -rf Python-${PYTHON_VERSION} Python-${PYTHON_VERSION}.tgz

# Install GDAL 3.0.4 from source
RUN wget http://download.osgeo.org/gdal/3.0.4/gdal-3.0.4.tar.gz && \
    tar -xvzf gdal-3.0.4.tar.gz && \
    cd gdal-3.0.4 && \
    ./configure && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd .. && \
    rm -rf gdal-3.0.4 gdal-3.0.4.tar.gz

# Set environment variables for GDAL headers
ENV CPLUS_INCLUDE_PATH=/usr/local/include
ENV C_INCLUDE_PATH=/usr/local/include

# Install Python packages
RUN pip3 install --upgrade pip wheel cython setuptools==57.5.0
RUN pip3 install GDAL==3.0.4
RUN pip3 install -r /opt/requirements.txt

# Install spatialindex
RUN wget http://download.osgeo.org/libspatialindex/spatialindex-src-1.7.1.tar.gz && \
    tar -xvf spatialindex-src-1.7.1.tar.gz && \
    cd spatialindex-src-1.7.1 && \
    ./configure && \
    make && \
    make install && \
    ldconfig && \
    cd .. && \
    rm -rf spatialindex-src-1.7.1 spatialindex-src-1.7.1.tar.gz

# Set locale
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# Install iRODS
RUN wget -qO - https://packages.irods.org/irods-signing-key.asc | apt-key add - && \
    echo "deb [arch=amd64] https://packages.irods.org/apt/ $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/renci-irods.list && \
    apt-get update -y && \
    apt-get upgrade -y && \
    wget -c http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb && \
    apt-get install -y ./libssl1.1_1.1.1f-1ubuntu2_amd64.deb && \
    rm -rf ./libssl1.1_1.1.1f-1ubuntu2_amd64.deb && \
    apt install -y irods-icommands

# Configure iRODS
RUN mkdir -p /root/.irods && \
    echo "{ \"irods_zone_name\": \"iplant\", \"irods_host\": \"data.cyverse.org\", \"irods_port\": 1247, \"irods_user_name\": \"$IRODS_USER\", \"irods_authentication_scheme\": \"PAM\", \"irods_password\": \"$IRODS_PASSWORD\" }" > /root/.irods/irods_environment.json

# Cleanup
RUN apt-get autoremove -y && apt-get clean

ENTRYPOINT [ "/usr/local/bin/python3.7", "/opt/transformation_gui.py" ]