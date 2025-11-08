# Control-and-Simulation-of-an-Induction-Motor
Description: This script performs the analysis, parameter calculation, and simulation of an induction motor using three control strategies: 1) Direct-on-line connection 2) Scalar (V/f) control 3) Vector (Field-Oriented) control
# 🌀 Induction Motor Control and Simulation (MATLAB/Simulink)

**Author:** Ing. Yevgeniy Makarenko  
**Year:** 2025  
**License:** Custom Non-Commercial License  
**Contact:** [e.makarenko.em49@gmail.com](mailto:e.makarenko.em49@gmail.com)

---

## 📘 Overview

This project focuses on the **analysis, modeling, and control** of a three-phase **induction motor** using MATLAB and Simulink.  
Three control methods are implemented and compared:

1. **Direct-On-Line (DOL) connection**  
   The motor is connected directly to the three-phase grid. Torque and current waveforms are analyzed under load.

2. **Scalar Control (V/f Control)**  
   The voltage and frequency are varied proportionally to maintain a constant ratio (U/f).  
   The response of speed and current is compared with the direct-on-line case.

3. **Vector Control (Field-Oriented Control, FOC)**  
   The most advanced method, allowing independent control of torque-producing and flux-producing currents.  
   Includes implementation of Park transformation, PI regulators, and slip speed calculation.

---

## ⚙️ Features

- Full simulation models for each control method in **Simulink**  
- Automatic visualization of results (currents, speed, torque, magnetic flux)  
- Clear parameter calculation section  
- Well-commented MATLAB code translated fully into English  
- Educational-friendly structure (for learning, teaching, and demonstration)

---

## 🧠 Theoretical Background

The induction motor model is based on standard equations of the asynchronous machine in the **dq (rotating)** reference frame.  
The vector control approach follows the **Rotor Flux-Oriented Control (RFOC)** scheme with decoupled torque and flux control loops.

---

## 📊 Simulation Results

Each simulation automatically produces plots:
- Speed response and reference comparison  
- Phase currents (a, b, c)  
- Electromagnetic torque  
- Flux and slip angular velocity  
- Components of stator current (*i₁x*, *i₁y*)

All results are shown in MATLAB figures and can be exported as PDF or images for reports.

---

## 📂 Repository Structure
📦 Induction_Motor_Control
├── model_am_priame.slx # Direct-on-line model
├── model_am_skalarne.slx # Scalar (V/f) control model
├── model_am_vektor.slx # Vector (FOC) control model
├── parametre_am.m # Main MATLAB script with calculations and plots
├── README.md # Project documentation
└── LICENSE.txt # Custom non-commercial license

---

