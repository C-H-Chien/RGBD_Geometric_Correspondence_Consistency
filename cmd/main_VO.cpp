#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

#include "../include/definitions.h"
#include "../include/Dataset.h"
#include "../include/Pipeline.h"

// =======================================================================================================
// main_VO: main function for LEMS VO pipeline
//
// ChangeLogs
//    Chien  24-01-16    Initially built on top of Hongyi's LEMS Visual Odometry framework.
//
//> (c) LEMS, Brown University
//> Chiang-Heng Chien (chiang-heng_chien@brown.edu)
// =======================================================================================================

//> usage: (Under the bin file) sudo ./main_VO --config_file=../config/tum.yaml

//> Define default values for the input argument
#if USE_GLOGS
DEFINE_string(config_file, "../config/tum.yaml", "config file path");
#endif

int main(int argc, char **argv) {

	//> Get input arguments
#if USE_GLOGS
	google::ParseCommandLineFlags(&argc, &argv, true);
#else
	//> Get input argument
	--argc; ++argv;
	std::string arg;
	int argIndx = 0, argTotal = 4;
	std::string FLAGS_config_file;

	if (argc) {
		arg = std::string(*argv);
		if (arg == "-h" || arg == "--help") {
			LOG_PRINT_HELP_MESSAGE;
			return 0;
		}
		else if (argc <= argTotal) {
		while(argIndx <= argTotal-1) {
			if (arg == "-c" || arg == "--config_file") {
				argv++;
				arg = std::string(*argv);
				FLAGS_config_file = arg;
				argIndx+=2;
				break;
			}
			else {
				LOG_ERROR("Invalid input arguments! Follow the instruction:");
				LOG_PRINT_HELP_MESSAGE;
				return 0;
			}
			argv++;
		}
		}
		else if (argc > argTotal) {
			LOG_ERROR("Too many input arguments! Follow the instruction:");
			LOG_PRINT_HELP_MESSAGE;
			return 0;
		}
	}
	else {
		LOG_PRINT_HELP_MESSAGE;
		return 0;
	}
#endif
	YAML::Node config_map;
	
	try {
		config_map = YAML::LoadFile(FLAGS_config_file);
#if SHOW_YAML_FILE_DATA
		std::cout << config_map << std::endl;
#endif
	}
	catch (const std::exception& e) {
		std::cerr << "Exception: " << e.what() << std::endl;
		std::cerr << "File does not exist!" << std::endl;
	}

	bool use_GCC_filter = true;

	//> Setup the dataset class pointer
	Dataset::Ptr dataset_ = Dataset::Ptr(new Dataset(config_map, use_GCC_filter));
    CHECK_EQ(dataset_->Init_Fetch_Data(), true);
	std::cout << "Total Number of Images in the Dataset Sequence: " << dataset_->Total_Num_Of_Imgs << std::endl;

	//> Pointers to the classes
	Frame::Ptr new_frame;
	Pipeline::Ptr vo_sys = Pipeline::Ptr(new Pipeline);

	//for (int fi = 0; fi < Total_Num_Of_Imgs; fi++) {
	for (int fi = 0; fi < 2; fi++) {
		
		//> Fetch the information of the next frame
		new_frame = dataset_->get_Next_Frame();
		if (new_frame == nullptr) LOG_ERROR("failed to fetch a frame");

		//vo_sys = Pipeline::Ptr(new Pipeline);
		bool success = vo_sys->Add_Frame(new_frame);
		//std::cout << "Number of SIFT Features: " << vo_sys->Num_Of_SIFT_Features << std::endl;

		if (vo_sys->get_Status() == PipelineStatus::STATUS_GET_AND_MATCH_SIFT) 
			continue;
		else {
			//> TODO: add RPE measurements here
		}
	}
	

	return 0;
}
