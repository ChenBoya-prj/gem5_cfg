From ubuntu:18.04
MAINTAINER Boya Chen "chenboya@hotmail.com"
ENV REFRESHED_AT 2020-05-12
RUN apt-get -yqq update --fix-missing && apt-get -yqq install libssl-dev gcc g++ build-essential git m4 scons zlib1g zlib1g-dev libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev python-dev python python-pydot python-six graphviz vim libtext-csv-perl cpu-checker
RUN mkdir -p /home
ADD ./gem5/ /home/gem5/
run cd /home/gem5 ; scons build/X86/gem5.opt -j4
run cd /home/gem5 ; scons build/ARM/gem5.opt -j4


