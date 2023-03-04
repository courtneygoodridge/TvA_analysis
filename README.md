This repo contains the data and analysis for the following manuscript:

Goodridge, C. M., Mole, C. D., Billington, J., Markkula, G., & Wilkie, R. M. (2022). Steering is initiated based on error accumulation. Journal of Experimental Psychology: Human Perception and Performance, 48(1), 64.

The ExpSimulation folder contains code (TrackSimulation.py) to simulate an observer moving at 8 m/s at varying angles of orientation relative to a straight road-line. This code calculates the visual angle and steering bias for orientations ranging from 0.5-2 degrees.

The Experiment_2_predictions script takes the visual angle and steeringbias metrics and generates predictions under Threshold and Accumulator frameworks for the conditions within the current experiment.

Experiment_2_analysis.Rmd is a script that pre-processes the experiment data  and generates reaction times, lateral position errors, and steering rates for each condition.

Experiment_2_modelling.Rmd takes the outputted dataframes from Experiment2_analysis.Rmd and fits multilevel models that are used to generate inferences on the data.

TvA_finished_figures_Exp2_only.Rmd creates figures from the data and what are used within the manuscript. 

Generating_Timecourse_Data.Rmd creates timecourse within the TvA_finished_figures_Exp2_only.Rmd for plotting average trajectories. 

