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
    // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Isometry3f model = Eigen::Isometry3f::Identity();
    model.rotate(Eigen::AngleAxisf(rotation_angle.z() / 180.0 * MY_PI, Eigen::Vector3f::UnitZ()));
    model.rotate(Eigen::AngleAxisf(rotation_angle.y() / 180.0 * MY_PI, Eigen::Vector3f::UnitY()));
    model.rotate(Eigen::AngleAxisf(rotation_angle.x() / 180.0 * MY_PI, Eigen::Vector3f::UnitX()));

    return model.matrix();
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float n = -zNear;
    float f = -zFar;
    float t = zNear * tan(eye_fov / 2.0 / 180.0 * MY_PI);
    float r = t * aspect_ratio;

    projection.coeffRef(0, 0) = n / r;
    projection.coeffRef(1, 1) = n / t;
    projection.coeffRef(2, 2) = (f + n) / (n - f);
    projection.coeffRef(2, 3) = -2 * f * n / (n - f);
    projection.coeffRef(3, 2) = 1;
    projection.coeffRef(3, 3) = 0;


    return projection;
}

int main(int argc, const char** argv)
{
    Eigen::Vector3f angle = {0, 0, 0};
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle.x() = std::stof(argv[2]); // -r by default
        angle.y() = std::stof(argv[3]); 
        angle.z() = std::stof(argv[4]); 
        if (argc == 6) {
            filename = std::string(argv[5]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    // 0-x 1-y 2-z
    int angle_flag = 2;
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

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'x')
        {
            angle_flag = 0;
        }
        else if (key == 'y')
        {
            angle_flag = 1;
        }
        else if (key == 'z')
        {
            angle_flag = 2;
        }

        if (key == 'a') {
            angle[angle_flag] += 5;
        }
        else if (key == 'd') {
            angle[angle_flag] -= 5;
        }
    }

    return 0;
}
