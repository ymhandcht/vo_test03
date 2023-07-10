#include<ros/ros.h>
#include<orb_features.hpp>
#include<coordCaculate.h>
#include<mindVision_init.h>
#include<iostream>
#include<orb_features_cuda.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<chrono>
using namespace std;
using namespace std::chrono;

/*PnP测试程序*/
// int main(int argc,char **argv)
// {
//     ros::init(argc,argv,"main");
//     Mat image1 = imread("/home/action/vo_test03/src/picture1/1.jpg");
//     Mat image2 = imread("/home/action/vo_test03/src/picture1/2.jpg");



  
// }
// bool nIfisCorrect(double n)
// {
//   return (n > 0 && abs(1 - n) <= 0.2) ? true : false;
// }


// /*
// int main(int argc,char *argv[])
// {
//     ros::init(argc,argv,"main");
//     //定义初始坐标为（0，0，0）
//     Point3f realCoor(0.0,0.0,0.0);
//     float angle1 = 0.0;
//     float angle2 = 0.0;
//     Point3f lastcastCoord(downWidth / 2, downHeigth / 2, 0); // 图像坐标中心点
//     MindVisionInit mindvision;
//     mindvision.init(); 
//     Mat image1 = mindvision.dstImage;  //首先记录第一张图像
//     Mat image2 = image1;
//     // Mat image1 = imread("/home/action/vo_test03/src/picture1/24.jpg");
//     // Mat image2 = imread("/home/action/vo_test03/src/picture1/22.jpg");

//     // cc.handle = cc.imu.imu_init(cc.handle, "/dev/pps_uart", B115200);
//     //  cc.storageImuAngle();
//     //  cout << "imu angle = " << cc.imuAngle << endl;
//     int num = 0;
//     while (ros::ok()) // ros::ok()
//     {
//       auto begin = std::chrono::steady_clock::now();
//       mindvision.updateImage();
//       cout << "num = " << num << endl;
//       auto start = std::chrono::steady_clock::now();
//       //cout << "更新图像用时：" << chrono::duration_cast<microseconds>(start - begin).count() << endl;
//       orb_cuda_features ocf(image1, image2);
//       ocf.orb_cuda_features_matching();
//       auto end1 = std::chrono::steady_clock::now();
//       auto time1 = chrono::duration_cast<microseconds>(end1 - start);
//         //cout << "特征匹配用时:" << time1.count() << "us" << endl;

//       CoordCaculate cc;
//       cc.initCoordCaculate();
//        // cc.storageImuAngle();
//         //将匹配的特征点放到容器里 分别存放第一幅和第二幅图像匹配的特征点
//       cc.storageFeaturePoints(ocf.matches,ocf.keyPoints_1,ocf.keyPoints_2);
//       auto end2 = std::chrono::steady_clock::now();
//       auto time2 = chrono::duration_cast<microseconds>(end2 - end1);
//         //cout << "存储（矫正）用时:" << time2.count() << "us" << endl;

//       cc.getAllMappedPoints();
//       auto end3 = std::chrono::steady_clock::now();
//       auto time3 = chrono::duration_cast<microseconds>(end3 - end2);
//         //cout << "计算位移用时:" << time3.count() << "us" << endl;

//       deque<Point3f> result = cc.mappedPoints;
//       cc.filterMappedPoints();
//       //cc.getAllRevolveAngles();
//       //cc.filterAngles();
//       //cout << "angleResult = " << cc.angleResult << endl;
//      // angle1 += cc.angleResult;
//       //cout << "vec1.size = " << cc.vec1.size() << endl;

//         // cout << "当前角度为:" << angle << endl;
//       auto end4 = std::chrono::steady_clock::now();
//       auto time4 = chrono::duration_cast<microseconds>(end4 - end3);
//         //cout << "滤波用时:" << time4.count() << endl;
//       Point3f coord = getNowCoord(lastcastCoord, cc.filterPoints);

//       realCoor+=coord;  //得到真实坐标

//       cout << "当前坐标为：" << realCoor << endl;

//        //  // cout << " imu角度为: " << cc.imuAngle << endl;
//        //  auto end5 = std::chrono::steady_clock::now();
//        //  auto time5 = chrono::duration_cast<microseconds>(end5 - end4);
//        //    //cout << "坐标更新用时:" << time5.count() << endl;

//           //更新图像
//         image2 = image1;
//         image1 = mindvision.dstImage;
//         auto end = steady_clock::now();
//         auto allTime = std::chrono::duration_cast<microseconds>(end - begin);
//         //cout << "程序总用时:" << allTime.count() << "us" << endl;
//         float dis = sqrt(coord.x*coord.x+coord.y*coord.y);  //距离单位是mm
//         float speed = 1000*dis/(float)(allTime.count());
//         num++;

//         //cout<<"当前移动速度是："<<speed<<"m/s"<<endl;

       
//     }

//     return 0;
// }
// */
// /*测试三角定位精度差测试*/
// int main(int argc,char *argv[])
// {
//     ros::init(argc,argv,"main");
//     //定义初始坐标为（0，0，0）
//     Point3f realCoor(0.0,0.0,0.0);
//     Point3f lastcastCoord(downWidth / 2, downHeigth / 2, 0); // 图像坐标中心点

//     // Mat image1 = imread("/home/action/vo_test03/src/picture1/24.jpg");
//     // Mat image2 = imread("/home/action/vo_test03/src/picture1/22.jpg");

//     // cc.handle = cc.imu.imu_init(cc.handle, "/dev/pps_uart", B115200);
//     //  cc.storageImuAngle();
//     //  cout << "imu angle = " << cc.imuAngle << endl;
//     int num = 1;
//     //问题 ：照片采集过慢
//     while (num<23) // ros::ok()
//     {
//       cout << "num = " << num << endl;
//       Mat image1 = imread("/home/action/vo_test03/src/picture1/" + to_string(num)+".jpg");
//       Mat image2 = imread("/home/action/vo_test03/src/picture1/" + to_string(num+2)+".jpg");

//       orb_cuda_features ocf(image1, image2);
//       ocf.orb_cuda_features_matching();
//       CoordCaculate cc;
//       cc.initCoordCaculate();

//       cc.storageFeaturePoints(ocf.matches,ocf.keyPoints_1,ocf.keyPoints_2);
   

//       cc.getAllMappedPoints();
   

//       deque<Point3f> result = cc.mappedPoints;
//       cc.filterMappedPoints();
//       //cc.getAllRevolveAngles();
//       //cc.filterAngles();
//       //cout << "angleResult = " << cc.angleResult << endl;
//      // angle1 += cc.angleResult;
//       //cout << "vec1.size = " << cc.vec1.size() << endl;

//       Point3f coord = getNowCoord(lastcastCoord, cc.filterPoints);

//       realCoor+=coord;  //得到真实坐标

//       cout << "当前坐标为：" << realCoor << endl;

//        //  // cout << " imu角度为: " << cc.imuAngle << endl;
//        //  auto end5 = std::chrono::steady_clock::now();
//        //  auto time5 = chrono::duration_cast<microseconds>(end5 - end4);
//        //    //cout << "坐标更新用时:" << time5.count() << endl;
//         num++;

//         //cout<<"当前移动速度是："<<speed<<"m/s"<<endl;

       
//     }

//     return 0;
// }

// /*采集图像程序*/

int main(int argc,char **argv)
{
  ros::init(argc,argv,"main");
  int picNum= 101;
  string picName = "";

  MindVisionInit mindvision;
  mindvision.init();

  while(true)
  {
    //auto begin = std::chrono::steady_clock::now();
    mindvision.updateImage();
    imshow("src", mindvision.dstImage);

    //auto start = std::chrono::steady_clock::now();
    //cout << "更新图像用时：" << chrono::duration_cast<microseconds>(start - begin).count() << endl;
    waitKey(1000);
    cout << "if save this picture? 1 yes others no" << endl;
    int flag;
    cin >> flag;
    if(flag == 1)
    {
      picNum++;
      picName = "/home/action/vo_test03/src/picture1/"+std::to_string(picNum)+".jpg";
      imwrite(picName,mindvision.dstImage);
    }
    else
      continue;
  }

  


  return 0;
}
// // #define a 0.785421
// // int main(int argc,char**argv)
// // {
// //   ros::init(argc, argv, "test");

// //   float b = 0.75432;
// //   float c = -0.75432;
// //   float result1 = 0.0;
// //   float result2 = 0.0;
// //   for (int i = 0; i < 100; i++)
// //   {
// //     b += 0.00001;
// //     c -= 0.00001;
// //     result1 += a * b;
// //     result2 += a * c;
// //   }
// //   cout << "result1 = " <<result1<< endl;
// //   cout << "result2 = " <<result2<< endl;

// //   return 0;
// // }

// #include<CameraParameter.h>
// Point2d pixelcam(const double &u,const double &v)
// {
//   Point2d point;
//   Camera camera;
//   point.x = (u - camera.camera_cx) / camera.camera_fx;
//   point.y = (v - camera.camera_cy) / camera.camera_fy;
//   return point;
// }

// int main(int argc,char** argv)
// {
//   ros::init(argc, argv, "main");
//   Mat image1 = imread("/home/action/vo_test03/src/picture1/3.jpg");
//   Mat image2 = imread("/home/action/vo_test03/src/picture1/4.jpg");
  
//   Point3f lastcastCoord(downWidth / 2, downHeigth / 2, 0);

//   orb_cuda_features ocf(image1,image2);
//   ocf.orb_cuda_features_matching();

//   CoordCaculate cc;
//   cc.initCoordCaculate();
//   cc.storageFeaturePoints(ocf.matches,ocf.keyPoints_1,ocf.keyPoints_2);

//   Mat homography_matrix;
//   homography_matrix = cv::findHomography(cc.vecc1, cc.vecc2, RANSAC, 3);
//   cout << "homography_matrix = " << homography_matrix << endl;
//   //旋转平移矩阵
//   vector<Mat> R;
//   vector<Mat> t;
//   vector<Mat> n;
//   Camera camera;
//   //相机内参
//   Mat camera_matrix = (Mat_<double>(3, 3) << camera.camera_fx, 0, camera.camera_cx, 0, camera.camera_fy, camera.camera_cy, 0, 0, 1);
//   cout << "camera_matrix = " << camera_matrix << endl;

//   int num = cv::decomposeHomographyMat(homography_matrix, camera_matrix, R, t, n);

//   //判断哪个解的平面法向量近似为（0，0，1）
//   /*****************************************/
//     int flag = nIfisCorrect(n[0].at<double>(2, 0)) ? 0 : (nIfisCorrect(n[1].at<double>(2, 0)) ? 1 : 2);
//   /*****************************************/
//   //验证对级约束
//   //归一化平面验证 不满足对级约束
//     Mat t_fdc = (Mat_<double>(3, 3) << 0, -t[flag].at<double>(2, 0), t[flag].at<double>(1, 0), t[flag].at<double>(2, 0), 0, -t[flag].at<double>(0, 0), -t[flag].at<double>(1, 0), t[flag].at<double>(0, 0), 0);//t反对称矩阵
    
//     // 将t归一化
//     // double r = t[flag].at<double>(2, 0);
//     // Mat t_corrected = (Mat_<double>(3, 1) << t[flag].at<double>(0, 0) / r, t[flag].at<double>(1, 0) / r, t[flag].at<double>(2, 0) / r);
//     // cout << "flag = " << flag << endl;
//     // cout << "R = " << R[flag] << endl;
//     // cout << "t = " << t[flag] << endl;
//     // cout << "t_corrected = " << t_corrected << endl;
//     // cout << "n = " << n[flag] << endl;
//     // Mat p1 = (Mat_<double>(3, 1) << 0, 0, 1);
//     // Mat p2 = R[0] * p1;
//     // cout << p2 << endl;

//     // cout << "R = " << R << endl;
//     // cout << "t = " << t << endl;

//     // cout << "num = " << num << endl;

//     // cc.getAllMappedPoints();
//     // cc.filterMappedPoints();

//     // Point3f coord = getNowCoord(lastcastCoord, cc.filterPoints);

//     // realCoor+=coord;  //得到真实坐标

//     // cout << "22-24当前坐标为：" << realCoor << endl;

//     return 0;
// }

// /*测试尺度因子*/
// // Mat cameraMatrix = (Mat_<double>(3,3)<<1046.657559*0.75,0.000000,658.051030*0.75,0.000000,1046.440390*0.75,525.818598*0.75,0.000000,0.000000,1.000000);
// // Mat distortionMatrix = (Mat_<double>(1,5)<<-0.107739,0.100439,-0.001195,-0.000826,0.000000);

// // void distortionCorrection(Mat &inputImage,Mat &outputImage)
// // {
// //     undistort(inputImage,outputImage,cameraMatrix,distortionMatrix);
// // }
// // int main(int argc,char **argv)
// // {
// //     ros::init(argc, argv, "main");
// //     Mat image1 = imread("/home/action/vo_test03/src/picture1/26.jpg");
// //     Point p1(905, 208);
// //     Point p2(905, 598);
// //     cv::circle(image1, p1, 3, Scalar(255, 0, 0), 2);
// //     cv::circle(image1, p2, 3, Scalar(0, 0, 255), 2);
    
// //     imshow("src", image1);
// //     waitKey(0);
// //     return 0;
// // }
