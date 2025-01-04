//> Macro definitions
#define USE_GLOGS                       (false)

#define OUTPUT_WRITE_PATH               std::string("Output_Results/")
#define OUTPUT_DATA_TYPE                std::string("TUM")  //> Either TUM or KITTI

//> Use for the TUM type dataset
#define ASSOCIATION_FILE_NAME           std::string("associate.txt")

//> Generic definitions
#define RANSAC_NUM_OF_ITERATIONS        (500)
#define EPSILON                         (1e-12)
#define REPROJ_ERROR_THRESH             (2)         //> in pixels

//> Geometric Correspondence Consistency
#define GCC_2D_THRESH                   (1)     //> pixels

#define DEPTH_GRAD_GAUSSIAN_SIGMA       (3)
#define GAUSSIAN_KERNEL_WINDOW_LENGTH   (2*4*DEPTH_GRAD_GAUSSIAN_SIGMA+1)

//> DEBUGGING PURPOSE
#define SHOW_YAML_FILE_DATA             (false)
#define WRITE_FEATURES_TO_FILE          (false)
#define WRITE_CORRESPONDENCES_TO_FILE   (false)
#define OPENCV_DISPLAY_FEATURES         (false)
#define OPENCV_DISPLAY_CORRESPONDENCES  (false)

//> SIFT parameters
#define SIFT_NFEATURES                  (0)
#define SIFT_NOCTAVE_LAYERS             (4)
#define SIFT_CONTRAST_THRESHOLD         (0.04)
#define SIFT_EDGE_THRESHOLD             (10)
#define SIFT_GAUSSIAN_SIGMA             (1.6)

#define LOWES_RATIO                     (0.8)        //> Suggested in Lowe's paper
#define K_IN_KNN_MATCHING               (2)

//> GCC Filter
#define ACTIVATE_GCC                    (true)
#define GCC_PATCH_HALF_SIZE             (3)          //> 3 pixels for r(\gamma_0) map

//> Print outs
#define LOG_INFO(info_msg)              printf("\033[1;32m[INFO] %s\033[0m\n", std::string(info_msg).c_str());
#define LOG_STATUS(status_)             printf("\033[1;35m[STATUS] %s\033[0m\n", std::string(status_).c_str());
#define LOG_ERROR(err_msg)              printf("\033[1;31m[ERROR] %s\033[0m\n", std::string(err_msg).c_str() );
#define LOG_TEST(test_msg)              printf("\033[1;30m[TEST] %s\033[0m\n", std::string(test_msg).c_str());
#define LOG_FILE_ERROR(err_msg)         printf("\033[1;31m[ERROR] File %s not found!\033[0m\n", std::string(err_msg).c_str() );
#define LOG_PRINT_HELP_MESSAGE          printf("Usage: ./main_VO [flag] [argument]\n\n" \
                                               "options:\n" \
                                               "  -h, --help         show this help message and exit\n" \
                                               "  -c, --config_file  path to the the configuration file\n");
