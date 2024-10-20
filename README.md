# Safe-Reinforcement-Learning-using-Symbolic-Logical-Programming-for-Autonomous-Highway-Driving

# Title
TOWARDS SAFE AUTONOMOUS DRIVING POLICIES USING A NEURO-SYMBOLIC DEEP REINFORCEMENT LEARNING APPROACH

## Abstract
<p align="justify"> The dynamic nature of driving environments and the presence of diverse road
users pose significant challenges for decision-making in autonomous driving.
Deep reinforcement learning (DRL) has emerged as a popular approach to tackle
this problem. However, the application of existing DRL solutions is mainly confined to simulated environments due to safety concerns, impeding their deployment in real-world. To overcome this limitation, this paper introduces a novel neuro-symbolic model-free DRL approach, called DRL with Symbolic Logics (DRLSL) that combines the strengths of DRL (learning from experience) and symbolic first-order logics (knowledge-driven reasoning) to enable safe learning in real-time interactions of autonomous driving within real environments. This innovative approach provides a means to learn autonomous driving policies by actively engaging with the physical environment while ensuring safety. We have implemented the DRLSL framework in autonomous driving using the highD dataset
and demonstrated that our method successfully avoids unsafe actions during both
the training and testing phases. Furthermore, our results indicate that DRLSL
achieves faster convergence during training and exhibits better generalizability to
new driving scenarios compared to traditional DRL methods.</p>

![](https://github.com/CAV-Research-Lab/Safe-Reinforcement-Learning-using-Symbolic-Logical-Programming-for-Autonomous-Highway-Driving/blob/main/videos/SDQN.png)

## Paper
Full pdf version of the paper is available in this [link](https://arxiv.org/pdf/2307.01316.pdf?).

## Video results
The following video show the performance of the autonomous vehicle in the highway scenario. The results show that not only is the driving completely safe, but also it is efficient.
You can see the result in this [video link](https://github.com/CAV-Research-Lab/Safe-Reinforcement-Learning-using-Symbolic-Logical-Programming-for-Autonomous-Highway-Driving/blob/main/videos/results_June6%2C2023.webm).
![](https://github.com/CAV-Research-Lab/Safe-Reinforcement-Learning-using-Symbolic-Logical-Programming-for-Autonomous-Highway-Driving/blob/main/videos/results_June6%2C2023.gif)

## How to Run:
All of the codes were verified on Ubuntu 20.04 (you may need to note the installation processes when using different Operating Systems):

1- create a virtual environment using `pycharm` or `virtualenv`.

2- activate the virtual environment using `source virtual_directory/bin/activate`.

3- install `swi-prolog` on your system using the following command:
```
sudo apt update
sudo apt install swi-prolog
```

3- use `pip` to install the requirement packages from the `requirements.txt` file.

4- clone the repository using the following command:
```
git clone https://github.com/CAV-Research-Lab/Safe-Reinforcement-Learning-using-Symbolic-Logical-Programming-for-Autonomous-Highway-Driving.git
```

5- run `train.py` in `DQN+SLP/` directory of the cloned repository to train the agent

## Code development

If you keep a close look at the `train.py` file, there are some initial parameters you need to change as you wish. The primary parameter is `SAFE`; when `SAFE=True`, we are running DQNSL program;otherwise, the program is a regular DQN. The parameters were set suitably, but if you need to change them, just act according to the comments in front of each parameter.

To set the hyperparameters of the deep Q-network, you can change the parameters in the beginning part of the `DQN.py` file. Moreover, you can find different reward functions in the `agent.py` file and set them manually if needed. The reward functions are not according to the article, but you can simply modify them. You can also add your own methods to the agent class.

We leveraged Prolog to implement symbolic first-order logics (FOL) in the context of autonomous highway drivings. The symbolic logical program (SLP) core (mentioned in the article) file, as termed `symbolic_logical_programming.pl`, can be found in the `prolog_files/` directory. If you need to develop the symbolic rules, you can add your own rules to the program. However, you should notice that the rules have been meticulously designed and there is no need for adding other rules.

## Reference
If you make use of the paper and code, please use the following bibliography to cite it:
```
@article{sharifi2023towards,
  title={towards safe autonomous driving policies using a neuro-symbolic deep reinforcement learning approach},
  author={Sharifi, Iman, and Yildirim, Mustafa, and Fallah, Saber},
  journal={arXiv preprint arXiv:2307.01316},
  year={2023}
}
```

