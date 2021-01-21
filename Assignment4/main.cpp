#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

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


float newPoint(float n1, float n2, double t)
{
    float diff = n2 - n1;

    return n1 + (diff * t);
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, double t) 
{
    float p0_x[4];
    float p0_y[4];
    float p1_x[3];
    float p1_y[3];    
    float p2_x[2];
    float p2_y[2];
    float p3_x;
    float p3_y;


    // TODO: Implement de Casteljau's algorithm
    for (int i = 0; i < 4; i++){
        p0_x[i] = control_points[i].x;
        p0_y[i] = control_points[i].y;
    }

    for (int i = 0; i < 3; i++){
        p1_x[i] = newPoint(p0_x[i], p0_x[i+1], t);
        p1_y[i] = newPoint(p0_y[i], p0_y[i+1], t);
    }

    for (int i = 0; i < 2; i++){
        p2_x[i] = newPoint(p1_x[i], p1_x[i+1], t);
        p2_y[i] = newPoint(p1_y[i], p1_y[i+1], t);
    }
    
    p3_x = newPoint(p2_x[0], p2_x[1], t);
    p3_y = newPoint(p2_y[0], p2_y[1], t);

    cv::Point2f newCtrlPt(p3_x, p3_y);
    
    std::cout << newCtrlPt << std::endl;

    return newCtrlPt;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto points = recursive_bezier(control_points, t);

        window.at<cv::Vec3b>(points.y, points.x)[1] = 255;
    }
}

int main() 
{
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

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
