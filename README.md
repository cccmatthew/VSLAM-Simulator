# VSLAM Simulator
Requirements\
Operating system: Ubuntu 16.04\
Programming language: Python 3.6.11
# Setup
Install Python 3.6.11\
Install pangolin and pyg2o \
Install dependencies on Python\
-Attached on requirements.txt\
-Install dependencies by running command: pip3 install -r requirements.txt\
Execute with command ‘/usr/bin/python YOURDIR/simulator/main.py’
# Pangolin Installation 
Reference link: https://github.com/uoip/pangolin \
git clone https://github.com/uoip/pangolin.git \
cd pangolin\
mkdir build\
cd build\
cmake ..\
make -j8\
cd ..\
python setup.py install
# pyg2o Installation
Reference link: https://github.com/uoip/g2opy \
git clone https://github.com/uoip/g2opy.git \
cd g2opy\
mkdir build\
cd build\
cmake ..\
make -j8\
cd ..
python setup.py install\

