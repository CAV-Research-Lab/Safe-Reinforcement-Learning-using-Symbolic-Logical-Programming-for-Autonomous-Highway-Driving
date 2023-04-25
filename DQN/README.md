# Interactive-Decision-Making-for-Autonomous-Driving-

Autonomous driving decision-making is a challenging task due to the inherent complexity and uncertainty in traffic. For example, adjacent vehicles may change their lane or overtake at any time to pass a slow vehicle or to help traffic flow. Anticipating the intention of surrounding vehicles, estimating their future states and integrating them into the decision-making process of an automated vehicle can enhance the reliability of autonomous driving in complex driving scenarios. This paper proposes a Prediction-based Deep Reinforcement Learning (PDRL) decision-making model that considers the manoeuvre intentions of surrounding vehicles in the decision-making process for highway driving. The model is trained using real traffic data and tested in various traffic conditions through a simulation platform. The results show that the proposed PDRL model improves the decision-making performance compared to a Deep Reinforcement Learning (DRL) model by decreasing collision numbers, resulting in safer driving.


## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install dependencies.

```bash
pip install
pygame==2.0.1
torch==1.11.0
numpy
tensorboard
```

## Usage
The code works based on HighD traffic data. The HighD traffic includes 60 different track data. Please check the website for the dataset (https://www.highd-dataset.com/)
You might need to extract road data before training and testing.


```python
train.py # To train agent
test.py # Testing
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.


## Citing the Repo
If you find the code useful in your research or wish to cite it, please use the following BibTeX entry.

```
@inproceedings{yildirim2022prediction,
  title={Prediction Based Decision Making for Autonomous Highway Driving},
  author={Yildirim, Mustafa and Mozaffari, Sajjad and McCutcheon, Luc and Dianati, Mehrdad and Tamaddoni-Nezhad, Alireza and Fallah, Saber},
  booktitle={2022 IEEE 25th International Conference on Intelligent Transportation Systems (ITSC)},
  pages={138--145},
  year={2022},
  organization={IEEE}
}
```
