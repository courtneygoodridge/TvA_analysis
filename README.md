**TvA_analysis**

Goodridge, C. M., Mole, C. D., Billington, J., Markkula, G., & Wilkie, R. M. (2022). Steering is initiated based on error accumulation. Journal of Experimental Psychology: Human Perception and Performance, 48(1), 64.

**Experimental design overview**

This experiment aimed to investigate how people initiate steering responses when drivers detect an error between their current heading and a target (in this experiment, a road-line). The heading of drivers was pertubed as various angles, left and right, relative to the target road-line. 

<img width="353" alt="image" src="https://github.com/courtneygoodridge/TvA_analysis/assets/44811378/ee0cb499-2dc8-4dab-a88b-c1e0a9754136">

The predictions of two cognitive frameworks were compared. A Threshold framework predicted that drivers would initiate a steering response when they reached a fixed distance from the road-line. Conversely, an Accumulator framework predicted drivers would accumulate perceptual error signals over time, and intiate a steering response once this accumulated quantity surpassed a fixed threshold. As is highlighted in the manuscript, this produces distinct predictions in the timing and magnitude of steering responses. 

**Code and analysis**

To replicate the analysis, people should focus on the `Experiment_2_modelling.Rmd` script. This take the `magnitudedata.csv` and fit multilevel models for each metric (reaction time, lateral position error, and steering rate). 


takes the outputted dataframes from Experiment2_analysis.Rmd and fits multilevel models that are used to generate inferences on the data.

The ExpSimulation folder contains code (`TrackSimulation.py`) to simulate an observer moving at 8 m/s at varying angles of orientation relative to a straight road-line. This code calculates the visual angle and steering bias for orientations ranging from 0.5-2 degrees.

The `Experiment_2_predictions.Rmd` script takes the visual angle and steering bias metrics and generates predictions under Threshold and Accumulator frameworks for the conditions within the current experiment.

`Experiment_2_analysis.Rmd` is a script that pre-processes the experimental data  and generates reaction times, lateral position errors, and steering rates for each trial.



TvA_finished_figures_Exp2_only.Rmd creates figures from the data and what are used within the manuscript. 

Generating_Timecourse_Data.Rmd creates timecourse within the TvA_finished_figures_Exp2_only.Rmd for plotting average trajectories. 

