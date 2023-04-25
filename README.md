# Safe-Reinforcement-Learning-using-Symbolic-Logical-Programming-for-Autonomous-Highway-Driving

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

6- run `train.py` in `SDQN/` directory of the cloned repository to train the agent
