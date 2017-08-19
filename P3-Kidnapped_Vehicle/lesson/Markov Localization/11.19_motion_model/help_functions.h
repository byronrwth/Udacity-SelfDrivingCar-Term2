//=================================================================================
// Name        : help_functions.h
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//=================================================================================

#ifndef HELP_FUNCTIONS_H_
#define HELP_FUNCTIONS_H_

#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "measurement_package.h"


class help_functions {
  public:

    //definition of one over square root of 2*pi:
    float ONE_OVER_SQRT_2PI = 1 / sqrt(2 * M_PI) ;

    //definition square:
    float squared(float x) {
        return x * x;
    }

    /*****************************************************************************
     * normpdf(X,mu,sigma) computes the probability function at values x using the
     * normal distribution with mean mu and standard deviation std. x, mue and
     * sigma must be scalar! The parameter std must be positive.
     * The normal pdf is y=f(x;mu,std)= 1/(std*sqrt(2pi)) e[ -(x−mu)^2 / 2*std^2 ]
    *****************************************************************************/
    float normpdf(float x, float mu, float std) {
        std::cout << "help_functions: normpdf(): x= " << x << " , mu= " << mu << " , std= " << std << std::endl;

        return (ONE_OVER_SQRT_2PI / std) * exp(-0.5 * squared((x - mu) / std));
    }

    //function to normalize a vector:
    std::vector<float> normalize_vector(std::vector<float> inputVector) {

        //declare sum:
        float sum = 0.0f;

        //declare and resize output vector:
        std::vector<float> outputVector ;
        outputVector.resize(inputVector.size());

        //estimate the sum:
        for (int i = 0; i < inputVector.size(); ++i) {
            sum += inputVector[i];
        }

        //normalize with sum:
        for (int i = 0; i < inputVector.size(); ++i) {
            outputVector[i] = inputVector[i] / sum ;
        }

        //return normalized vector:
        return outputVector ;
    }


    /* Reads map data from a file.
     * @param filename Name of file containing map data.
     */
    inline bool read_map_data(std::string filename, map& map) {

        // Get file of map:
        std::ifstream in_file_map(filename.c_str(), std::ifstream::in);
        // Return if we can't open the file.
        if (!in_file_map) {
            return false;
        }

        //declare single line of map file:
        std::string line_map;

        //run over each single line:
        while (getline(in_file_map, line_map)) {

            std::istringstream iss_map(line_map);

            //declare landmark values and ID:
            float landmark_x_f;
            int id_i;

            //read data from current line to values::
            iss_map >> id_i ;
            iss_map >> landmark_x_f ;


            //declare single_landmark:
            map::single_landmark_s single_landmark_temp ;

            //set values
            single_landmark_temp.id_i = id_i ;
            single_landmark_temp.x_f  = landmark_x_f;

            //push_back in landmark list of map_1d:
            map.landmark_list.push_back(single_landmark_temp);

        }
        return true;
    }


    /* Reads measurements from a file.
     * @param filename Name of file containing measurement  data.
     */
    inline bool read_measurement_data(std::string filename_control,
                                      std::string filename_obs,
                                      std::vector<MeasurementPackage>& measurement_pack_list) {
        //get file of measurements:
        std::ifstream in_file_control(filename_control.c_str(), std::ifstream::in);
        if (!in_file_control) {
            return false;
        }
        //declare single line of measurement file:
        std::string line;

        int count = 1 ;

        //run over each single line:
        while (getline(in_file_control, line)) {

            //declare measurement package:
            MeasurementPackage meas_package;

            std::istringstream iss(line);

            //declare position values:
            float delta_x_f;

            //read data from line to values:
            iss >> delta_x_f;

            std::cout << "help_functions: read controls delta_x_f= " << delta_x_f <<  std::endl;
            //set control information:
            meas_package.control_s_.delta_x_f = delta_x_f ; // all 1 !!

            //read observations for each control information:
            char str_obs[1024];

            //define file name of observations for current control/position info:
            sprintf(str_obs, "%sobservations_%06i.txt", filename_obs.c_str(), count);
            std::string in_file_name_observation = std::string(str_obs);


            //get file of observations:
            std::ifstream in_file_observation(in_file_name_observation.c_str(),
                                              std::ifstream::in);
            if (!in_file_observation) {
                return false;
            }

            std::string line_obs;

            //run over each single line:
            while (getline(in_file_observation, line_obs)) {

                std::istringstream iss_obs(line_obs);

                //declare observation values:
                float distance_f;

                //read data from line to values:
                iss_obs >> distance_f;
                std::cout << "help_functions: read observation_s_ distance_f= "<< distance_f <<  std::endl;

                //set observation information:
                meas_package.observation_s_.distance_f.push_back(distance_f);
            }

            std::cout << "help_functions: in_file_observation is of size " << meas_package.observation_s_.distance_f.size() << std::endl; // 0, no observ!
            //push_back single package in measurement list:
            measurement_pack_list.push_back(meas_package);

            //increase counter for observation files:
            count++;
        }

        std::cout << "help_functions: count for No. of controls: " << count - 1 << std::endl; //T = 14

        return true;
    }

    inline bool compare_data(std::string filename_gt,
                             std::vector<float>& result_vec) {
        /*****************************************************************************
         *  print/compare results:                                                 *
         *****************************************************************************/
        //get GT data:
        //define file name of map:

        std::vector<float> gt_vec;

        //get file of map:
        std::ifstream in_file_gt(filename_gt.c_str(), std::ifstream::in);

        //declare single line of map file:
        std::string line_gt;

        //run over each single line:
        while (getline(in_file_gt, line_gt)) {

            std::istringstream iss_gt(line_gt);
            float gt_value;

            //read data from current line to values::
            iss_gt >> gt_value  ;
            gt_vec.push_back(gt_value);

        }
        float error_sum;
        float belief_sum;
        error_sum  = 0.0f;
        belief_sum = 0.0f;
        std::cout << "..................................................." << std::endl;
        std::cout << ".............result_vec..----> Results <----................." << std::endl;
        std::cout << "..................................................." << std::endl;

        for (int i = 0; i <  result_vec.size(); ++i) {
            error_sum += (gt_vec[i] - result_vec[i]) * (gt_vec[i] - result_vec[i]);
            belief_sum += result_vec[i] ;
            std::cout << std::fixed << std::setprecision(5) << "bel_x=" << i << ":" << "\t"
                      << result_vec[i] << "\t"
                      << "ground truth:" << "\t"
                      << gt_vec[i] << "\t" << std::endl ;
        }
        std::cout << "..................................................." << std::endl;
        std::cout << std::fixed << std::setprecision(5) << "sum bel:"    << "\t" << belief_sum << std::endl;
        std::cout << "..................................................." << std::endl;
        std::cout << std::fixed << std::setprecision(5) << "sqrt error sum:     " << "\t" << sqrt((error_sum)) << std::endl;
        std::cout << "..................................................." << std::endl;
        std::cout << "..................................................." << std::endl;
        return true;

    }




};

#endif /* HELP_FUNCTIONS_H_ */

/*  without observation, i.e. posterior_obs == 1
...................................................
...............----> Results <----.................
...................................................
bel_x=0:    0.00000 ground truth:   0.00000 
bel_x=1:    0.00000 ground truth:   0.00000 
bel_x=2:    0.00000 ground truth:   0.00000 
bel_x=3:    0.00000 ground truth:   0.00000 
bel_x=4:    0.00000 ground truth:   0.00000 
bel_x=5:    0.00000 ground truth:   0.00000 
bel_x=6:    0.00000 ground truth:   0.00000 
bel_x=7:    0.00000 ground truth:   0.00000 
bel_x=8:    0.00001 ground truth:   0.00000 
bel_x=9:    0.00002 ground truth:   0.00000 
bel_x=10:   0.00005 ground truth:   0.00000 
bel_x=11:   0.00013 ground truth:   0.00000 
bel_x=12:   0.00028 ground truth:   0.00000 
bel_x=13:   0.00058 ground truth:   0.00000 
bel_x=14:   0.00111 ground truth:   0.00000 
bel_x=15:   0.00199 ground truth:   0.00000 
bel_x=16:   0.00333 ground truth:   0.00000 
bel_x=17:   0.00523 ground truth:   0.00000 
bel_x=18:   0.00770 ground truth:   0.00000 
bel_x=19:   0.01066 ground truth:   0.00000 
bel_x=20:   0.01390 ground truth:   0.00000 
bel_x=21:   0.01714 ground truth:   0.00000 
bel_x=22:   0.02009 ground truth:   0.00000 
bel_x=23:   0.02250 ground truth:   0.00000 
bel_x=24:   0.02424 ground truth:   0.00001 
bel_x=25:   0.02529 ground truth:   0.00004 
bel_x=26:   0.02566 ground truth:   0.00011 
bel_x=27:   0.02539 ground truth:   0.00029 
bel_x=28:   0.02452 ground truth:   0.00072 
bel_x=29:   0.02307 ground truth:   0.00165 
bel_x=30:   0.02120 ground truth:   0.00350 
bel_x=31:   0.01913 ground truth:   0.00690 
bel_x=32:   0.01723 ground truth:   0.01263 
bel_x=33:   0.01589 ground truth:   0.02146 
bel_x=34:   0.01541 ground truth:   0.03383 
bel_x=35:   0.01589 ground truth:   0.04950 
bel_x=36:   0.01723 ground truth:   0.06721 
bel_x=37:   0.01913 ground truth:   0.08470 
bel_x=38:   0.02120 ground truth:   0.09907 
bel_x=39:   0.02307 ground truth:   0.10754 
bel_x=40:   0.02452 ground truth:   0.10834 
bel_x=41:   0.02539 ground truth:   0.10130 
bel_x=42:   0.02566 ground truth:   0.08790 
bel_x=43:   0.02529 ground truth:   0.07079 
bel_x=44:   0.02424 ground truth:   0.05290 
bel_x=45:   0.02250 ground truth:   0.03670 
bel_x=46:   0.02009 ground truth:   0.02362 
bel_x=47:   0.01714 ground truth:   0.01411 
bel_x=48:   0.01390 ground truth:   0.00782 
bel_x=49:   0.01066 ground truth:   0.00403 
bel_x=50:   0.00770 ground truth:   0.00192 
bel_x=51:   0.00523 ground truth:   0.00085 
bel_x=52:   0.00333 ground truth:   0.00035 
bel_x=53:   0.00199 ground truth:   0.00013 
bel_x=54:   0.00111 ground truth:   0.00005 
bel_x=55:   0.00058 ground truth:   0.00002 
bel_x=56:   0.00028 ground truth:   0.00000 
bel_x=57:   0.00013 ground truth:   0.00000 
bel_x=58:   0.00006 ground truth:   0.00000 
bel_x=59:   0.00004 ground truth:   0.00000 
bel_x=60:   0.00006 ground truth:   0.00000 
bel_x=61:   0.00013 ground truth:   0.00000 
bel_x=62:   0.00028 ground truth:   0.00000 
bel_x=63:   0.00058 ground truth:   0.00000 
bel_x=64:   0.00110 ground truth:   0.00000 
bel_x=65:   0.00196 ground truth:   0.00000 
bel_x=66:   0.00328 ground truth:   0.00000 
bel_x=67:   0.00510 ground truth:   0.00000 
bel_x=68:   0.00742 ground truth:   0.00000 
bel_x=69:   0.01009 ground truth:   0.00000 
bel_x=70:   0.01280 ground truth:   0.00000 
bel_x=71:   0.01518 ground truth:   0.00000 
bel_x=72:   0.01681 ground truth:   0.00000 
bel_x=73:   0.01739 ground truth:   0.00000 
bel_x=74:   0.01681 ground truth:   0.00000 
bel_x=75:   0.01518 ground truth:   0.00000 
bel_x=76:   0.01281 ground truth:   0.00000 
bel_x=77:   0.01011 ground truth:   0.00000 
bel_x=78:   0.00748 ground truth:   0.00000 
bel_x=79:   0.00523 ground truth:   0.00000 
bel_x=80:   0.00356 ground truth:   0.00000 
bel_x=81:   0.00254 ground truth:   0.00000 
bel_x=82:   0.00220 ground truth:   0.00000 
bel_x=83:   0.00254 ground truth:   0.00000 
bel_x=84:   0.00356 ground truth:   0.00000 
bel_x=85:   0.00523 ground truth:   0.00000 
bel_x=86:   0.00748 ground truth:   0.00000 
bel_x=87:   0.01011 ground truth:   0.00000 
bel_x=88:   0.01281 ground truth:   0.00000 
bel_x=89:   0.01518 ground truth:   0.00000 
bel_x=90:   0.01681 ground truth:   0.00000 
bel_x=91:   0.01739 ground truth:   0.00000 
bel_x=92:   0.01681 ground truth:   0.00000 
bel_x=93:   0.01518 ground truth:   0.00000 
bel_x=94:   0.01280 ground truth:   0.00000 
bel_x=95:   0.01009 ground truth:   0.00000 
bel_x=96:   0.00742 ground truth:   0.00000 
bel_x=97:   0.00510 ground truth:   0.00000 
bel_x=98:   0.00327 ground truth:   0.00000 
bel_x=99:   0.00194 ground truth:   0.00000 
...................................................
sum bel:    1.00000
...................................................
 rse   :        0.22621
...................................................
...................................................
*/