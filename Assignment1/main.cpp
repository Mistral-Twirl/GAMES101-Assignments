#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(Eigen::Vector3f rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f rotationMatX;
    Eigen::Matrix4f rotationMatY;
    Eigen::Matrix4f rotationMatZ;
    
    int i=0;
    while (i<3){
        rotation_angle[i] = rotation_angle[i] / 180 * MY_PI;
        i++;
    }

    rotationMatX << 1, 0, 0, 0,
                    0, cos(rotation_angle[0]), -1*sin(rotation_angle[0]), 0,
                    0, sin(rotation_angle[0]), cos(rotation_angle[0]), 0,
                    0, 0, 0, 1; 

    rotationMatY << cos(rotation_angle[1]), sin(rotation_angle[1]), 0, 0,
                    0, 1, 0, 0,
                    -1*sin(rotation_angle[1]), cos(rotation_angle[1]), 0, 0,
                    0, 0, 0, 1; 

    rotationMatZ << cos(rotation_angle[2]), -1*sin(rotation_angle[2]), 0, 0,
                   sin(rotation_angle[2]), cos(rotation_angle[2]), 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1; 

    model = rotationMatX * rotationMatY * rotationMatZ * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    eye_fov = eye_fov / 180 * MY_PI;
    float t = abs(zNear) * tan(eye_fov/2);
    float b = -1 * t;
    float r = t * aspect_ratio;
    float l = -1 * r;

    Eigen::Matrix4f orthoMat[3];
    Eigen::Matrix4f orthopersMat;


    orthoMat[0] << 1, 0, 0, -(r+l)/2,
                   0, 1, 0, -(t+b)/2,
                   0, 0, 1, -(zNear+zFar)/2,
                   0, 0, 0, 1;
    orthoMat[1] << 2/(r-l), 0, 0, 0,
                   0, 2/(t-b), 0, 0,
                   0, 0, 2/(zNear-zFar), 0,
                   0, 0, 0, 1;
    orthoMat[2] = orthoMat[1] * orthoMat[0];


    orthopersMat << zNear, 0, 0, 0,
                    0, zNear, 0, 0,
                    0, 0, zNear+zFar, -(zNear*zFar),
                    0, 0, 1, 0;

    projection = orthoMat[2] * orthopersMat * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    Eigen::Vector3f angle = {0, 0, 0};
    bool command_line = false;
    std::string filename = "output.png";
/*
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }
*/
    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    /*if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
*/

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        // std::cout << "frame count: " << frame_count++ << '\n';
/*
        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
*/
        angle[0] += 3;
        angle[1] += 6;
        angle[2] += 9;
        //std::cout << angle[0]<< angle[1]<<angle[2]<< std::endl;
    }
    
    return 0;
}
