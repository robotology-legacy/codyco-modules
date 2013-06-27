% Script for plotting the data dumped from the force/torque sensors of iCub
% It plots the 6 components (forces and moments) of the force/torque
% sensors in legs and feet. It computes the mean of the measurements across
% the experiment (through a sliding window).
%
% Date: 27-06-2013
% Author: Serena Ivaldi (serena.ivaldi@isir.upmc.fr)
% License: GPL
% Coyright:CODYCO Consortium (www.codyco.eu)

leftFootFTS = load('leftFootFTS/data.log');
rightFootFTS = load('rightFootFTS/data.log');
leftLegFTS = load('leftLegFTS/data.log');
rightLegFTS = load('rightLegFTS/data.log');

interv = 50;

[LF_startm, LF_endm, LF_maxm, LF_minm] = analyse_one_fts(leftFootFTS,interv,'Left Foot FTS');
[RF_startm, RF_endm, RF_maxm, RF_minm] = analyse_one_fts(rightFootFTS,interv,'Right Foot FTS');
[LL_startm, LL_endm, LL_maxm, LL_minm] = analyse_one_fts(leftLegFTS,interv,'Left Leg FTS');
[RL_startm, RL_endm, RL_maxm, RL_minm] = analyse_one_fts(rightLegFTS,interv,'Right Leg FTS');