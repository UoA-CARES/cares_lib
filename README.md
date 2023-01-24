# cares_lib
Generally useful functions and methods for projects throughout the CARES team. 
Methods existing in this repository are useful across a broad range of work we are doing and not specific to any given project.

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites
What things you need to install the software and how to install them

```
cd PATH_TO_CARES_LIB_FOLDER/cares_lib
pip install .
```

### Installing
Run the following command to install the library onto the local computer. 

```
python3 setup.py install --user 
```

# Contributing
To add new features and functions to this library first consider if the feature is generic or specific to a given project.
If the feature is generically useful, pull the package and create a branch to add in the new features.
Once complete, use a pull request into the main branch to have the addition reviewed before finalisation into the main branch.

# Libraries
Brief overview of useful tools in this library, for specific details see the individual package. 

## utils
General purpose utility functions, typically interfacing or conversion functions between various packages.

## vision
General purpose utility classes or functions for doing computer vision.

## dynamixel
Classes and functions for control of dynamixel servos

## slack-bot
Slack bot control code for automated communication and interaction with slack channels.