% This creates a .mat file with all the distance maps. This is done in
% order to speed up the loading of all of this data every time we run the
% MPC code.
%
% In order to use this, all the distance files must be in this same folder,
% with the file of this function and nothing else.
%
% e.g.
% - distance_maps (this folder's name)
%   - generate_dMaps_file.m
%   - distance_objects_2.csv
%   - distance_objects_3.csv
%   - distance_objects_4.csv
%   - distance_objects_5.csv
%   ...
%
% Running this script will generate a file called dMaps.mat in this folder,
% which is used by the MPC code.


clear; clc;
dMap2 = csvread("csv/distance_object_2.csv");
dMap3 = csvread("csv/distance_object_3.csv");
dMap4 = csvread("csv/distance_object_4.csv");
dMap5 = csvread("csv/distance_object_5.csv");
dMap6 = csvread("csv/distance_object_6.csv");
dMap7 = csvread("csv/distance_object_7.csv");
dMap8 = csvread("csv/distance_object_8.csv");
dMap9 = csvread("csv/distance_object_9.csv");
save dMaps
