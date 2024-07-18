#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
int num_control_points = 4;

/**
 * @brief 鼠标的callback函数 选择控制点
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param userdata 
 */
void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < num_control_points) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

/**
 * @brief 使用4个控制点的解析表达式绘制贝塞尔曲线
 * 
 * @param points 
 * @param window 
 */
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // 深拷贝control_points 
    std::vector<cv::Point2f> tmp_points = control_points;
    while (tmp_points.size() > 1) {
        for (int i = 0; i < tmp_points.size() - 1; i++) 
            tmp_points[i] = (1 - t) * tmp_points[i] + t * tmp_points[i + 1];
        tmp_points.pop_back();
    }
    return tmp_points.front();
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        // 
        int x = static_cast<int>(point.x),
            y = static_cast<int>(point.y);
        float dx = point.x - x,
              dy = point.y - y;
        // 通过dx dy计算到达像素中心的距离，作为颜色的权重
        float factor = std::max(1.0f - sqrt(pow(((dx - 0.5f) / 0.5f), 2) + pow(((dy - 0.5) / 0.5f), 2)), 0.5);
        window.at<cv::Vec3b>(y, x)[1] = (int)(255 * factor);
    }

}

int main(int argc, char **argv) 
{
    // 根据命令行读入参数设定控制点数目
    if (argc == 2) 
        num_control_points = std::stoi(argv[1]);
    std::cout << "Number of control points: " << num_control_points << std::endl;
 
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == num_control_points) 
        {
            naive_bezier(control_points, window);
            cv::imwrite("naive_bezier_curve_" + std::to_string(num_control_points) + ".png", window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            // 将num_control_points转化为string作为图像后缀
            cv::imwrite("my_bezier_curve_" + std::to_string(num_control_points) + ".png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
