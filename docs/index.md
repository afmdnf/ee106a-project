---
order: 0
---

# Introduction

![Workspace Setup](/assets/setup_images/setup_1a.jpg)

## Goal

Our project goal was to identify various kitchenware scattered about a table, classify each object into designated types (such as cups, plates and utensils), and instruct Baxter to pick up each object, manipulate it, and sort it based on its type.

## Project Description

Our system uses an external RealSense depth camera to obtain pointclouds within the Baxter workspace, isolates only the pointclouds containing kitchenware, classifies each available object based on its unique point distribution, and sends that data to Baxter. Baxter subsequently executes a series of commands to pick each object up, manipulate it, and then sort it into a designated location within the workspace. Getting this system to work reliably involves tight coordination between several subsystems, each of which requires navigating various interesting problems:
* The RealSense camera must be localized with respect to the Baxter.
* Kitchenware within an image must be confidently distinguished from the table behind it, while accounting for noise and visual artifacts.
* The number of unique objects on the table must be determined quickly and accurately, without computationally-intensive algorithms.
* Each object must be classified purely based on the shape of its pointcloud, with no other major informative assumptions (such as each object's color).
* Object types and poses must be constantly communicated to Baxter, since it is constantly and actively manipulating the workspace.
* The Baxter must be able to use each object's classification to make informed decisions about how to pick it up, what to do with it, and where to take it, while correcting for inevitable error in each object's actual location and orientation.

## Real-World Applications

Our project was created with the idea of designing a robot that could manually wash, dry and sort dirty kitchenware when scattered about a table, a task that, if automated, could drastically decrease both time and labor costs, both in the home and in the food service industry. Furthermore, we envision our system extending beyond kitchenware, to a multitude of tasks that could require manual manipulation and organization of objects within a workspace, such as with assembly lines in industrial manufacturing.
