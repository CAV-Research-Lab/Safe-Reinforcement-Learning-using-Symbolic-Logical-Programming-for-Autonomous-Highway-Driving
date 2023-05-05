# Safe-Reinforcement-Learning-using-Symbolic-Logical-Programming-for-Autonomous-Highway-Driving

## Abstract

The rapidly changing driving environments and the presence of other road users make decision-making in
autonomous highway driving a challenging problem. Deep reinforcement learning (DRL) has been a popular approach to
address this problem, but current DRL solutions are limited to simulation environments due to safety concerns, making their
transfer to the real environment a challenge. To overcome this limitation, this paper proposes a novel safe DRL framework that
combines the benefits of symbolic logical programming (SLP) and DRL for safe learning during real-time interaction in a real
environment. The proposed method provides a novel solution to obtain autonomous driving policies that can be trained through
interaction with real envirounment while the safety is assured. We have implemented the proposed method on the highD dataset
and proved that with our method, the agent did not involve in unsafe actions in both the training and test scenarios. Moreover,
we have shown that the method outperformed the DRL agent without SLP.

## How to Run:
Running steps on Ubuntu 20.04:

1- create a virtual environment using `pycharm` or `virtualenv`

2- activate the virtual environment using `source virtual_directory/bin/activate`

3- install `swi-prolog` on your system using the following command:
```
sudo apt update
sudo apt install swi-prolog
```

3- use `pip` to install the requirement packages from the `requirements.txt` file

4- you should make some changes to `PySwip` package:
    
+ if swi-prolog version is 9.0.4, then you should find `core.py` file in the virtual environment libraries (`lib`) and replace `PL_version` parameter with `PL_version_info`. 
+ if swi-prolog version is older like 8.4.3, then skip this step.

+ in `prolog.py` file, paste the following code in line 157:
```    
@classmethod 
def reconsult(cls, filename, catcherrors=False):
next(cls.query(filename.join(["reconsult('", "')"]), catcherrors=catcherrors))
```
5- clone the repository using the following command:
```
git clone https://github.com/CAV-Research-Lab/Safe-Reinforcement-Learning-using-Symbolic-Logical-Programming-for-Autonomous-Highway-Driving.git
```

6- run `train.py` in `DQN+SLP/` directory of the cloned repository to train the agent
