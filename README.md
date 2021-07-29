# icp-ceres
ICP based on ceres

首先，利用icp-svd计算初值（可参考高翔的ＳＬＡＭ1４讲）
然后，利用ceres优化ｉｃｐ

src下命令
$ mkdir build && cd build
cmake ..
make -j4
