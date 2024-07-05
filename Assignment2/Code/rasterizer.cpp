// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector4f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    float px = x, py = y,
          ax = _v[0].x(), ay = _v[0].y(),
          bx = _v[1].x(), by = _v[1].y(),
          cx = _v[2].x(), cy = _v[2].y();
    
    Eigen::Vector2f P{px, py}, A{ax, ay}, B{bx, by}, C{cx, cy};

    // 实现一个lambda函数，用于计算叉乘
    auto sign = [](Eigen::Vector2f& v1, Eigen::Vector2f& v2, Eigen::Vector2f& v3) -> int {
        float epsilon = 1e-7;
        Eigen::Vector2f v21 = v2 - v1, v31 = v3 - v1;
        float cross = v21.x() * v31.y() - v31.x() * v21.y();
        if (cross > epsilon) return 1;
        else if (cross < -epsilon) return -1;
        else return 0;
    };
    // AB x AP
    int sign1 = sign(A, B, P);
    // BC x BP
    int sign2 = sign(B, C, P);
    // CA x CP
    int sign3 = sign(C, A, P);
    
    return ((sign1 == sign2) && (sign2 == sign3));
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    // zNear - 0.1
    // zFar - 50
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        // 取出三角形的三个顶点，转换为齐次坐标后，进行MVP变换
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        // 透视除法
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        // 视口变换 顶点坐标-> ndc坐标系
        for (auto & vert : v)
        {
            // [-1, 1] -> [0, w]
            vert.x() = 0.5*width*(vert.x()+1.0);
            // [-1, 1] -> [0, h]
            vert.y() = 0.5*height*(vert.y()+1.0);
            // [-1, 1] -> [zNear, zFar]
            vert.z() = vert.z() * f1 + f2;
        }

        // 设置三角形顶点坐标
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        // 设置三角形顶点颜色
        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        // 光栅化
        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    // Step1: Find bounding box of triangle
    float x_minf = __FLT_MAX__, y_minf = __FLT_MAX__, x_maxf = __FLT_MIN__, y_maxf = __FLT_MIN__;

    for (auto vi : v)
    {
        x_minf = std::min(vi.x(), x_minf);
        x_maxf = std::max(vi.x(), x_maxf);
        y_minf = std::min(vi.y(), y_minf);
        y_maxf = std::max(vi.y(), y_maxf);
    }
    int x_min = (int)floor(x_minf - 0.5f),
        y_min = (int)floor(y_minf - 0.5f),
        x_max = (int)ceil(x_maxf + 0.5f),
        y_max = (int)ceil(y_maxf + 0.5f);
    // Step2: Traverse pixels in bounding box and determine if pixel is inside triangle
    for (int i = x_min; i < x_max; i++)
    {
        for (int j = y_min; j < y_max; j++)
        {
            if (insideTriangle(i, j, v.data()))
            {
                // Step3: If pixel is inside triangle, calculate z value using barycentric coordinates
                //        If z value is less than the value in the depth buffer, update the depth buffer and set the pixel color
                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                
                if (z_interpolated < depth_buf[get_index(i, j)])
                {
                    // Step4: Interpolate color using barycentric coordinates and set the pixel color
                    depth_buf[get_index(i, j)] = z_interpolated;
                    col_buf[get_index(i, j)].push_back(t.getColor());
                }
                Eigen::Vector3f p{i, j, 1.0f};
                set_pixel(p, t.getColor());
            }
        }
    }
    
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on