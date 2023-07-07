#include<ros/ros.h>
#include<orb_features_cuda.h>

void knnMatches2(Mat &query,Mat &train,vector<DMatch>&matches)
{
    vector<vector<DMatch>> knn_matches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.knnMatch(query,train,knn_matches,2);

    float min_dist = FLT_MAX;
    for (int r = 0; r < knn_matches.size(); ++r)
    {
        if (knn_matches[r][0].distance > 0.6*knn_matches[r][1].distance)
            continue;

        float dist = knn_matches[r][0].distance;
        if (dist < min_dist) min_dist = dist;
    }

    matches.clear();
    for (size_t r = 0; r < knn_matches.size(); ++r)
    {
        if (
            knn_matches[r][0].distance > 0.6*knn_matches[r][1].distance ||
            knn_matches[r][0].distance > 5 * max(min_dist, 10.0f) 
            )
            continue;
        matches.push_back(knn_matches[r][0]);
    }

}
orb_cuda_features::orb_cuda_features(Mat img1,Mat img2)
{

    
    this->orignalImage_1 = img1;
    this->orignalImage_2 = img2;
    
    this->G_img1.upload(orignalImage_1);
    this->G_img2.upload(orignalImage_2);
    
    cuda::cvtColor(G_img1, G_imggray1, COLOR_BGR2GRAY);
    cuda::cvtColor(G_img2, G_imggray2, COLOR_BGR2GRAY);
    
    
}

void orb_cuda_features::orb_cuda_features_matching()
{
    auto orb_cuda_detector = cuda::ORB::create(2000, 1.2f, 12, 31, 0, 2, 0, 31, 20, true);
    Ptr<cv::cuda::DescriptorMatcher> G_matcher = cv::cuda::DescriptorMatcher::createBFMatcher(4);

    orb_cuda_detector->detectAndComputeAsync(G_imggray1,cuda::GpuMat(),G_keypoints1,G_descriptors1);

    orb_cuda_detector->convert(G_keypoints1,keyPoints_1);
   

    G_descriptors1.convertTo(G_descriptors1_32F,CV_32F);
  
    Mat descriptors1(G_descriptors1);

    orb_cuda_detector->detectAndComputeAsync(G_imggray2,cuda::GpuMat(),G_keypoints2,G_descriptors2);
    orb_cuda_detector->convert(G_keypoints2,keyPoints_2);
    G_descriptors1.convertTo(G_descriptors2_32F,CV_32F);
    

    Mat descriptors2(G_descriptors2);

    G_matcher->match(G_descriptors1_32F,G_descriptors2_32F,this->matches);
    knnMatches2(descriptors1,descriptors2,matches);
    // cout<<"好的匹配点个数："<<matches.size()<<endl;
    // drawMatches(orignalImage_1,keyPoints_1,orignalImage_2,keyPoints_2,matches,this->matchImage);
    // imshow("src",matchImage);
}


//图像去畸变
//vins mono 方法
//p_u 是输入坐标点 d_u是畸变后坐标位置
void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u)
{
    
    double k1 = Camera::camera_k1;
    double k2 = Camera::camera_k2;
    double p1 = Camera::camera_p1;
    double p2 = Camera::camera_p2;

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);//x方
    my2_u = p_u(1) * p_u(1);//y方
    mxy_u = p_u(0) * p_u(1);//xy
    rho2_u = mx2_u + my2_u;//畸变公式里的r方

    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    //畸变程度计算
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}


//形参 p是待矫正畸变的点 P是迭代矫正后的点
void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P)
{
    double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    //double lambda;

    // Lift points to normalised plane
    // 将像素坐标投影到归一化相机坐标系，得到归一化的坐标（含有畸变）
    // p(0)p(1)是像素坐标
    Camera camera;
    float m_inv_K11 = 1.0 / camera.camera_fx;
    float m_inv_K13 = -camera.camera_cx/camera.camera_fx;
    float m_inv_K22 = 1.0 / camera.camera_fy;
    float m_inv_K23 = -camera.camera_cy / camera.camera_fy;
    mx_d = m_inv_K11 * p(0,0) + m_inv_K13;
    my_d = m_inv_K22 * p(1,0) + m_inv_K23;
    
    int m_noDistortion = 0;
    if (m_noDistortion)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
        {
            int n = 8; // 迭代次数
            // d_u表示以当前像素点畸变后的坐标,即B',C'等点
            // 将代表畸变距离变量，用于迭代，直到迭代点在误差允许范围内接近目标点，即可近似求得目标坐标
            Eigen::Vector2d d_u;
            distortion(Eigen::Vector2d(mx_d, my_d), d_u);
            // Approximate value
            // 第一次得到靠近目标点的一个坐标值
            // 减d_u()的目的是为了得到临时点的坐标，因为临时点在靠近目标点的一侧。即图中的C D ...点
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);

            // 迭代n次找靠近目标点的临时点，认为迭代n次后就认为接近目标点了（在允许的误差范围内）
            for (int i = 1; i < n; ++i)
            {
                distortion(Eigen::Vector2d(mx_u, my_u), d_u);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1); 
            }
        }
    }

    // Obtain a projective ray
    // 去畸变后的真实投影在归一化相机坐标系上的坐标
    P << mx_u, my_u, 1.0;
}
