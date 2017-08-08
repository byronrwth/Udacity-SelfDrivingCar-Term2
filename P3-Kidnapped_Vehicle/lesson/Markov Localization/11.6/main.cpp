//=================================================================================
// Name        : 1d_bayesian filter
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//=================================================================================

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include "measurement_package.h"
#include "map.h"
#include "help_functions.h"

using namespace std;

int main() {

    /******************************************************************************
     *  declaration:
     *****************************************************************************/

    //define example: 01, 02, 03, 04
    string example_string = "01";

    //declare map:
    map map_1d;

    //declare measurement list:
    std::vector<MeasurementPackage> measurement_pack_list;

    //declare helpers:
    help_functions helper;

    //define file names:
    char in_file_name_ctr[1024];
    char in_file_name_obs[1024];
    char in_file_name_gt[1024];

    /******************************************************************************
     *  read map and measurements:                                               *
     *****************************************************************************/
    //read map:
    helper.read_map_data("data/map_1d.txt", map_1d);

    //define file name of controls:
    sprintf(in_file_name_ctr, "data/example%s/control_data.txt",
            example_string.c_str());

    //define file name of observations:
    sprintf(in_file_name_obs, "data/example%s/observations/",
            example_string.c_str());

    //read in data to measurement package list:
    helper.read_measurement_data(in_file_name_ctr,
                                 in_file_name_obs,
                                 measurement_pack_list);

    /*****************************************************************************
         *  Coding quiz 1: just print out map infos and measurement package:     *
    ******************************************************************************/

    /////////////
    //Add code://
    /////////////
    printf("*************************************************\n");
    printf("Please print out map and measurement information!\n");
    printf("*************************************************\n");
    ////////////
    //Results://
    ////////////

    std::cout << "..................................................." << std::endl;
    std::cout << "..................................................." << std::endl;
    std::cout << "............----> Coding quiz 1  <----............." << std::endl;
    std::cout << "..................................................." << std::endl;
    std::cout << "..................................................." << std::endl;

    //print out map:
    std::cout << "Print out the map landmarks:" << endl;


    for (int i = 0; i < map_1d.landmark_list.size(); i++) {
        std::cout << "ID: " << map_1d.landmark_list[i].id_i << "\t"
                  << "value in x: " << map_1d.landmark_list[i].x_f << std::endl;
    }
    std::cout << "..................................................." << std::endl;
    std::cout << "..................................................." << std::endl;

    //print out the controls and the observations:
    std::cout << "Print out the measurement packages:" << endl;

    for (int i = 0; i < measurement_pack_list.size(); i++) {

        std::cout << "Step " << i << " includes the move "
                  << measurement_pack_list[i].control_s_.delta_x_f
                  <<  "[m] in driving direction " << std::endl;

        //run over observations:
        if (measurement_pack_list[i].observation_s_.distance_f.size() < 1) {
            std::cout << "   No observations in step " << i << std::endl;
        } else {

            std::cout << "   Number of Observations in current step: "
                      << measurement_pack_list[i].observation_s_.distance_f.size()
                      << std::endl;

            for (int j = 0; j < measurement_pack_list[i].observation_s_.distance_f.size(); j++ ) {
                std::cout << "   Distance to a landmark: "
                          <<  measurement_pack_list[i].observation_s_.distance_f[j]
                          <<  " m" << std::endl;
            }
        }
        std::cout << "..................................................." << std::endl;
    }
    return 0;

    /*
Print out the map landmarks:
ID: 1   value in x: 9
ID: 2   value in x: 15
ID: 3   value in x: 25
ID: 4   value in x: 31
ID: 5   value in x: 59
ID: 6   value in x: 77
...................................................
...................................................
Print out the measurement packages:
Step 0 includes the move 1[m] in driving direction 
    No observations in step 0
...................................................
Step 1 includes the move 1[m] in driving direction 
    No observations in step 1
...................................................
Step 2 includes the move 1[m] in driving direction 
    No observations in step 2
...................................................
Step 3 includes the move 1[m] in driving direction 
    No observations in step 3
...................................................
Step 4 includes the move 1[m] in driving direction 
    No observations in step 4
...................................................
Step 5 includes the move 1[m] in driving direction 
    No observations in step 5
...................................................
Step 6 includes the move 1[m] in driving direction 
    No observations in step 6
...................................................
Step 7 includes the move 1[m] in driving direction 
    No observations in step 7
...................................................
Step 8 includes the move 1[m] in driving direction 
    No observations in step 8
...................................................
Step 9 includes the move 1[m] in driving direction 
    No observations in step 9
...................................................
Step 10 includes the move 1[m] in driving direction 
    No observations in step 10
...................................................
Step 11 includes the move 1[m] in driving direction 
    No observations in step 11
...................................................
Step 12 includes the move 1[m] in driving direction 
    No observations in step 12
...................................................
Step 13 includes the move 1[m] in driving direction 
    No observations in step 13
...................................................

    */
}