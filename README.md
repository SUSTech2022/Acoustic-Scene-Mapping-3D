# Introduction
This repository is an upgraded version of our previous 2D framework: [I-ASM](https://github.com/SUSTech2022/Acoustic-Scene-Mapping), now extended to 3D acoustic scene mapping.

## Repository Structure
- **`exp_data/`**: Contains all experimental data, organized by different test arrangements. Each arrangement has its own folder with relevant data files.
- **`functions/`**: Contains all custom functions required to run the main code.
- **`mbss_locate/`**: Includes the Sound Source Localization (SSL) MATLAB toolbox, [*Multichannel BSS Locate*](https://gitlab.inria.fr/bass-db/mbss_locate), which is used to estimate the **direction-of-arrival (DoA)** of **multiple sources** in a multichannel audio signal recorded by a microphone array.
- **`ASM_offline_3D.m`**: Contains code to evaluate offline ASM.
- **`ASM_online_3D.m`**: Contains code to perform online ASM.
- **`cycle.m`**: Contains code of the _Filtering-Clustering-Implicit Association_ Cycle.

## Usage Guide

### Offline ASM Evaluation
To visualize offline ASM results:

1. Open the **`ASM_offline.m`** script.
2. Modify the script to select the desired arrangement and SSL method.
3. Run the script to visualize the offline mapping results.

### Online ASM Implementation
To perform online ASM during robot movement:

1. As the robot moves, save audio files (in `.wav` format) to the **`exp_data/audio/`** folder and the corresponding pose data in **`exp_data/pose/pose_theta.xlsx`**.
2. Run the **`ASM_online.m`** script to start the online ASM process.
3. The mapping will update automatically when new audio data is detected in the audio folder.

## Note
Ensure that the required MATLAB Toolbox is installed before running the scripts.

## Contact
For questions or feedback, please contact linya.fu@outlook.com.

