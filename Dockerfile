FROM ubuntu:18.04

WORKDIR /opt
COPY . /opt

USER root
ARG DEBIAN_FRONTEND=noninteractive
ARG PYTHON_VERSION=3.7.1
ENV IRODS_USER=phytooracle
ENV IRODS_PASSWORD=mac_scanalyzer
RUN apt-get -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false update -y

RUN apt-get update 
RUN apt-get install -y wget \
                       gdal-bin \
                       libgdal-dev \
                       libspatialindex-dev \
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
                       tk-dev \
                       libgdbm-dev \
                       libc6-dev \
                       liblzma-dev \
                       libsm6 \
                       libxext6 \
                       libxrender-dev \
                       libgl1-mesa-dev \
                       curl \
                       gnupg2

# Download and extract Python sources
RUN cd /opt \
    && wget https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tgz \                                              
    && tar xzf Python-${PYTHON_VERSION}.tgz

# Build Python and remove left-over sources
RUN cd /opt/Python-${PYTHON_VERSION} \ 
    && ./configure --with-ensurepip=install \
    && make install \
    && rm /opt/Python-${PYTHON_VERSION}.tgz /opt/Python-${PYTHON_VERSION} -rf

# Install GDAL
RUN add-apt-repository ppa:ubuntugis/ubuntugis-unstable
RUN apt-get update
RUN apt-get install -y libgdal-dev
RUN pip3 install --upgrade pip
RUN pip3 install --upgrade wheel
RUN pip3 install cython
RUN pip3 install --upgrade cython
RUN pip3 install setuptools==57.5.0
RUN pip3 install GDAL==3.0.4
RUN pip3 install -r /opt/requirements.txt

RUN wget http://download.osgeo.org/libspatialindex/spatialindex-src-1.7.1.tar.gz
RUN tar -xvf spatialindex-src-1.7.1.tar.gz
RUN cd spatialindex-src-1.7.1/ && ./configure && make && make install
RUN ldconfig                       
RUN add-apt-repository ppa:ubuntugis/ppa
RUN export CPLUS_INCLUDE_PATH=/usr/include/gdal
RUN export C_INCLUDE_PATH=/usr/include/gdal

RUN apt-get install -y locales && locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

#Install iRODS
RUN wget -qO - https://packages.irods.org/irods-signing-key.asc | apt-key add -
RUN echo "deb [arch=amd64] https://packages.irods.org/apt/ $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/renci-irods.list

RUN apt-get update -y \
    && apt-get upgrade -y

RUN wget -c \
    http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb
RUN apt-get install -y \
    ./libssl1.1_1.1.1f-1ubuntu2_amd64.deb
RUN rm -rf \
    ./libssl1.1_1.1.1f-1ubuntu2_amd64.deb

RUN apt install -y irods-icommands
RUN mkdir -p /root/.irods
RUN echo "{ \"irods_zone_name\": \"iplant\", \"irods_host\": \"data.cyverse.org\", \"irods_port\": 1247, \"irods_user_name\": \"$IRODS_USER\", \"irods_authentication_scheme\": \"PAM\", \"irods_password\": \"$IRODS_PASSWORD\" }" > /root/.irods/irods_environment.json
RUN apt-get autoremove -y
RUN apt-get clean

# RUN echo "deb [arch=amd64] https://packages.irods.org/apt/ $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/renci-irods.list && \
#     curl -s https://packages.irods.org/irods-signing-key.asc | apt-key add -

# RUN apt-get update && apt-get install -y irods-runtime irods-icommands

# RUN mkdir -p /etc/irods && \
#     touch /etc/irods/service_account.config && \
#     echo "phytooracle" > /etc/irods/service_account.config && \
#     echo "mac_scanalyzer" >> /etc/irods/service_account.config

ENTRYPOINT [ "/usr/local/bin/python3.7", "/opt/transformation_gui.py" ]
