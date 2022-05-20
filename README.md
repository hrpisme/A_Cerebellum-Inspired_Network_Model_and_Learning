# A_Cerebellum-Inspired_Network_Model_and_Learning



MBCI-puma560 要是碰到奇异点或者初始位置距离太远，都会导致Wout*p太大，导致的问题就是经过tanh函数后结果都是1


MFCI将论文里的轨迹复现完成，但是对于大一点的轨迹，完成度不是很好，猜测可能原因是因为MLP的训练不够完美
