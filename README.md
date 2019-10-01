# Medical-Simulation-Haptics
## Abstract
This research work aims to develop a virtual environment appropriate for
interactive (real-time) simulations of medical and surgical interventions.
These simulations have many applications including training of surgeons,
planning of interventions (for rehearsal and for the validation of a therapy)
or providing assistance during a real surgery. Nevertheless, in order to obtain an accurate and realistic simulation of a procedure, we have to simulate
with precision the deformation of anatomical structures and the interactions
between rigid medical devices and soft tissues. In addition, the simulation
must be interactive and computed in real-time to keep the gesture of the
physician in the loop of the simulation. The main goal of this research work
is to design a simulation which combines a high level of accuracy for the
deformations with low level of computation time.


First, we present the Co-rotational FEM method which is based on finite
element methods (FEM) and computes the biomechanics of anatomical soft
tissues. Second, we analyse the dynamic equations of the deformable bodies and we present the Lagrange multipliers approach which handles the
interactions in our virtual environment. Due to the fact that visual feedback
is not enough, physicians use haptic feedback for guidance. For this reason, we analyse the key concepts of the algorithms dedicated to real-time
haptic rendering. Furthermore, we present a detailed analysis of our implementation including a system of human organs such as: lungs, bronchus,
diaphragm and thorac, which the user can interact with, in real-time. Finally, we report the results derived from conducted experiments which prove
the validity of our application. We propose future work that will establish
medical simulation in the field of medicine.

## Virtual Environment

The virtual environment includes the human lungs with dark pink, diahragm with dark red, bronchus with white and thorac with transparent grey.

![Καταγραφή18](https://user-images.githubusercontent.com/43147324/65963213-f0d61280-e462-11e9-92d3-3ec9b7d96120.PNG)

There is a multimodel representation for each organ. Dark pink is the visual model, blue is the internal (FEM) model and yellow is the collision model of the human lung.

![Καταγραφή15](https://user-images.githubusercontent.com/43147324/65963684-f8e28200-e463-11e9-967a-91224f8a99cd.PNG)

## Interactions and Deformations

Figures (a), (b), (c) show the deformation caused by pushing
different parts of the left lung’s surface. In picture (d) we can see the
deformation produced by grasping and pulling the human lung. Transparent pink color used to represent the undeformed state of the soft organ and dark pink to highlight the deformed state.

## Comparison with more precise models

In this section, we replace the internal model of human left lung with two coarser and three more high resolution models. The
number of elements each object includes is presented in the following table. The
purpose of this section is to observe the improvement of the precision of
the deformation using more analytic FEM models ( with greater number
of elements) . However, using more precise internal models decreases the
computational performance of our application leading to no real-time results.


## Comparison with FEBIO
