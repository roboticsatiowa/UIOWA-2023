# UIOWA-2023

## Getting Started

### Installing ROS

It is highly recommended to use a [Ubuntu linux](https://ubuntu.com/desktop) operating system.

If needed a [virtual machine](https://www.virtualbox.org/wiki/Downloads) will work

Follow the installation instructions found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Setting up python environment

1. Create python virtual environment: 
```python3 -m venv venv```

2. Enable virtual environment (venv): 
```source venv/bin/activate```

    - Note: venv can be disabled by running ```deactivate``` at any time

3. Install dependencies (must have ros 2 already installed): 
```pip install -r requirements.txt```

    - Note: requirements.txt can be updated with new dependencies by running ```pip freeze > requirements.txt```

### Running the code

1. Launch the rover with ```ros2 launch rover_launch.xml```

2. Launch the laptop with ```ros2 launch home_launch.xml```

## Documentation

- [ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials.html)
