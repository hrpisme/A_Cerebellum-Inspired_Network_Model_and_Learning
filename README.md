# A_Cerebellum-Inspired_Network_Model_and_Learning



MBCI-puma560 要是碰到奇异点或者初始位置距离太远，都会导致Wout*p太大，导致的问题就是经过tanh函数后结果都是1


MFCI将论文里的轨迹复现完成，但是对于大一点的轨迹，完成度不是很好，猜测可能原因是因为MLP的训练不够完美

MBCI------------复现puma560的MBCI方案

MFCI------------复现puma560的MFCI方案

MFCI_MLP--------训练puma560的MFCI方案的神经网络

net.mat.mat-----训练好的神经网络

train.mat.mat---对应的训练数据

UR5-------------matlab复现的UR5的MBCI方案，论文用的rvep，目前不会rvep


