#include<ros/ros.h>
#include<iostream>
#include "orb_extractor1.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<chrono>
#include "vfc.h"
#include<orb_features_cuda.h>
#include<mindVision_init.h>
#include<coordCaculate.h>
#include<orb_features_cuda.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
using namespace cv;
using namespace std;
using namespace XIAOC;


// int main(int argc,char **argv)
// {
//     ros::init(argc, argv, "main");
//     MindVisionInit mindvision;
//     mindvision.init();

//     int nfeatures = 1000;
//     int nlevels = 1;
//     float fscaleFactor = 1.0;
//     float fIniThFAST = 40;
//     float fMinThFAST = 10;

//     Mat imagesrc = mindvision.dstImage;
//     Mat imagecur = imagesrc;

//     Point3f realCoor(0.0,0.0,0.0);
//     Point3f lastcastCoord(downWidth / 2, downHeigth / 2, 0); // 图像坐标中心点

    
//      while(ros::ok())
//     {
//         mindvision.updateImage();
//         imagesrc = mindvision.dstImage;
//         Mat grayImgsrc, mask;
//         cv::cvtColor( imagesrc, grayImgsrc, 6 );
//         Mat grayImgcur,maskcur;
//         cv::cvtColor( imagecur, grayImgcur, 6 );
//         ORBextractor *pORBextractor;
//         pORBextractor = new ORBextractor( nfeatures, fscaleFactor, nlevels, fIniThFAST, fMinThFAST );
//         Mat srcdesc;
//         vector<KeyPoint> srckps;
//         (*pORBextractor)( grayImgsrc, mask, srckps, srcdesc );
//         Mat curdesc;
//         vector<KeyPoint> curkps;
//         (*pORBextractor)( grayImgcur, maskcur, curkps, curdesc );
//          BFMatcher matcher_bf(NORM_HAMMING, true); //使用汉明距离度量二进制描述子，允许交叉验证
//         vector<DMatch> Matches_bf;
//         matcher_bf.match(srcdesc, curdesc, Matches_bf);
//         if(Matches_bf.size() == 0)
//             cout << "no match" << endl;

//         vector<Point2f> X;
//         vector<Point2f> Y;
//         X.clear();
//         Y.clear();

//         for(int i=0;i<Matches_bf.size();i++){
//             int index1 = Matches_bf.at(i).queryIdx;
//             int index2 = Matches_bf.at(i).trainIdx;
//             X.push_back(srckps.at(index1).pt);
//             Y.push_back(curkps.at(index2).pt);
//         }

//         VFC myvfc;
//         myvfc.setData(X, Y);
//         myvfc.optimize();
//         vector<int> matchIdx = myvfc.obtainCorrectMatch();

//         // 筛选正确的匹配
//         std::vector< DMatch > Matches_VFC;
//         for (unsigned int i = 0; i < matchIdx.size(); i++) {
//         int idx = matchIdx[i];
//             Matches_VFC.push_back(Matches_bf[idx]);
//         }

//         //cout << Matches_VFC.size() << endl;

//         CoordCaculate cc;
//         cc.initCoordCaculate();
//         //将匹配的特征点放到容器里 分别存放第一幅和第二幅图像匹配的特征点
//         cc.storageFeaturePoints(Matches_VFC,srckps,curkps);
      
//         cc.getAllMappedPoints();

//         deque<Point3f> result = cc.mappedPoints;
//         cc.filterMappedPoints();
      
//         Point3f coord = getNowCoord(lastcastCoord, cc.filterPoints);

//         realCoor+=coord;  //得到真实坐标

//         cout << "当前坐标为：" << realCoor << "  "<< realCoor.x*realCoor.x+realCoor.y*realCoor.y << endl;

//         imagecur = imagesrc;
//     }
// }

/*PnP测试*/
#define Z 0.53
int main(int argc,char **argv)
{
        ros::init(argc,argv,"main");
       
        Mat K_Matrix = (Mat_<float>(3, 3) << Camera::camera_fx, 0, Camera::camera_cx, 0, Camera::camera_fy, 0, 0, 0, 1);
        Mat distMatrix = (Mat_<float>(1, 5) << Camera::camera_k1, Camera::camera_k2, Camera::camera_k3, Camera::camera_p1, Camera::camera_p2);

        Eigen::Matrix<float, 3, 3> R_result = Eigen::Matrix3f::Identity();
        
        Eigen::Matrix<float,3,1> t_result;
        t_result<<0,0,0;

        int nfeatures = 1000;
        int nlevels = 1;
        float fscaleFactor = 1.0;
        float fIniThFAST = 40;
        float fMinThFAST = 10;
        string picName = "";
        int picNum = 1;
        MindVisionInit mindvision;
        mindvision.init();
        Mat imagesrc = mindvision.dstImage;
        Mat imagecur = imagesrc;

        while(ros::ok())
        {
            mindvision.updateImage();
            imagesrc = mindvision.dstImage;
            Mat grayImgsrc, mask;
            cv::cvtColor( imagesrc, grayImgsrc, 6 );
            Mat grayImgcur,maskcur;
            cv::cvtColor( imagecur, grayImgcur, 6 );
            ORBextractor *pORBextractor;
            pORBextractor = new ORBextractor( nfeatures, fscaleFactor, nlevels, fIniThFAST, fMinThFAST );
            Mat srcdesc;
            vector<KeyPoint> srckps;
            (*pORBextractor)( grayImgsrc, mask, srckps, srcdesc );
            Mat curdesc;
            vector<KeyPoint> curkps;
            (*pORBextractor)( grayImgcur, maskcur, curkps, curdesc );
            BFMatcher matcher_bf(NORM_HAMMING, true); //使用汉明距离度量二进制描述子，允许交叉验证
            vector<DMatch> Matches_bf;
            matcher_bf.match(srcdesc, curdesc, Matches_bf);
            if(Matches_bf.size() == 0)
                cout << "no match" << endl;

            vector<Point2f> X;
            vector<Point2f> Y;
            X.clear();
            Y.clear();

            for(int i=0;i<Matches_bf.size();i++){
                int index1 = Matches_bf.at(i).queryIdx;
                int index2 = Matches_bf.at(i).trainIdx;
                X.push_back(srckps.at(index1).pt);
                Y.push_back(curkps.at(index2).pt);
            }

            VFC myvfc;
            myvfc.setData(X, Y);
            myvfc.optimize();
            vector<int> matchIdx = myvfc.obtainCorrectMatch();

            // 筛选正确的匹配
            std::vector< DMatch > Matches_VFC;
            for (unsigned int i = 0; i < matchIdx.size(); i++) {
                int idx = matchIdx[i];
                Matches_VFC.push_back(Matches_bf[idx]);
            }
            //第一幅图像地图点
            vector<cv::Point3f> vSrc_mappoints ;
        
            vector<cv::Point2f> vCur;
            for (int i = 0; i < Matches_VFC.size(); ++i)
            {
                int index1 = Matches_VFC[i].queryIdx;
                int index2 = Matches_VFC[i].trainIdx;
                float x1 = srckps[index1].pt.x;
                float y1 = srckps[index1].pt.y;
            
                float z1 = Z;
                float x = z1 * (x1 - Camera::camera_cx) / Camera::camera_fx;
                float y = z1 * (y1 - Camera::camera_cy) / Camera::camera_fy;
                float z = z1;
            
                cv::Point3f v1(x, y, z);
                float x2 = curkps[index2].pt.x;
                float y2 = curkps[index2].pt.y;
                cv::Point2f v2(x2, y2);
                vCur.emplace_back(v2);
                vSrc_mappoints.push_back(v1);
       
            // cout << vSrc_mappoints[i] << endl;
            }
            //第二幅图像像素点
            Mat R,t;
            cv::solvePnPRansac(vSrc_mappoints, vCur, K_Matrix, distMatrix, R, t);
            Mat Rvec;
            Mat_<float> Tvec;
            R.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
            t.convertTo(Tvec, CV_32F); // 平移向量转换格式 

            Mat_<float> rotMat(3, 3);
            Rodrigues(Rvec, rotMat);
            // 旋转向量转成旋转矩阵
            // cout << "rotMat" << endl << rotMat << endl << endl;
            // cout << t << endl;
        

        // Mat P_oc;
        // P_oc = -rotMat.inv() * Tvec;
        // // 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
        // cout << "P_oc" << endl << P_oc << endl;
        Eigen::Matrix<float, 3, 3> m1;
        Eigen::Matrix<float, 3, 1> m2;
        // cout << "R = " << R << endl;
        //cout << "t = " << Tvec.at<float>(0,0) << endl;

        for (int i = 0; i < 3; i++)
        {
            m2(i, 0) = Tvec.at<float>(i, 0);

            for (int j = 0; j < 3; j++)
            {
                m1(i, j) = rotMat.at<float>(i, j);
            }
        }
        R_result = m1*R_result;
        t_result += m2;
        
        
        
        cout << "R_result = " << R_result << endl;
        cout << "t_result = " << t_result << endl;
        imagecur = imagesrc;
        }    
        
    
        

        
        return 0;
    
}
    // -----grid based orb extractor
    
    // Mat grayImgsrc,mask;
    // cv::cvtColor( imagesrc, grayImgsrc, 6 );

  
    // orb extractor initialize
    // chrono::steady_clock::time_point time1 = chrono::steady_clock::now();
    // ORBextractor *pORBextractor;
    // pORBextractor = new ORBextractor( nfeatures, fscaleFactor, nlevels, fIniThFAST, fMinThFAST );
    // cout << "ORBextractor initialize finished!" << endl;

    // // orb extractor
    // cout << "Extract orb descriptors..." << endl;
    


    // chrono::steady_clock::time_point time2 = chrono::steady_clock::now();
    // chrono::duration<double> time = chrono::duration_cast<chrono::duration<double>>(time2 - time1);
    // cout<<"均匀化特征点计算用时："<<time.count()*1000<<"ms"<<endl;


    // cout << "Extract orb descriptors finished!" << endl;
    // cout << "The number of keypoints are = " << srckps.size() << endl;

    // draw keypoints in output image
    // Mat outImgsrc;
    // drawKeypoints( grayImgsrc, srckps, outImgsrc, cv::Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    // imshow( "GridOrbKpsImg", outImgsrc );
    // waitKey( 0 );
    // cout << "Finished! Congratulations!" << endl;


    // // ----original orb extractor for comparation
    // // orb initialization 
    // cout << "Using original orb extractor to extract orb descriptors for comparation." << endl;
    // Ptr<ORB> orb_ = ORB::create( 1000, 1.2f, 8, 19 );

    // // orb extract
    // vector<KeyPoint> orb_kps;
    // Mat orb_desc;
    // orb_->detectAndCompute( grayImgsrc, mask, orb_kps, orb_desc );

    // // draw keypoints in output image
    // Mat orbOutImg;
    // drawKeypoints( grayImgsrc, orb_kps, orbOutImg, cv::Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    // imshow( "OrbKpsImg", orbOutImg );
    // waitKey(0);

    // Mat grayImgcur,maskcur;
    // cv::cvtColor( imagecur, grayImgcur, 6 );

  
    // orb extractor initialize
    // chrono::steady_clock::time_point time1 = chrono::steady_clock::now();
    




    // 第二幅图像提取特征点，均匀化
    // pORBextractor = new ORBextractor( nfeatures, fscaleFactor, nlevels, fIniThFAST, fMinThFAST );
    // cout << "ORBextractor initialize finished!" << endl;

    // // orb extractor
    // cout << "Extract orb descriptors..." << endl;
    // Mat curdesc;
    // vector<KeyPoint> curkps;
    // (*pORBextractor)( grayImgcur, maskcur, curkps, curdesc );


    // chrono::steady_clock::time_point time2 = chrono::steady_clock::now();
    // chrono::duration<double> time = chrono::duration_cast<chrono::duration<double>>(time2 - time1);
    // cout<<"均匀化特征点计算用时："<<time.count()*1000<<"ms"<<endl;


    // cout << "Extract orb descriptors finished!" << endl;
    // cout << "The number of keypoints are = " << srckps.size() << endl;

    // draw keypoints in output image
    // Mat outImgcur;
    // drawKeypoints( grayImgcur, curkps, outImgcur, cv::Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    // imshow( "GridOrbKpsImg1", outImgcur );
    // waitKey( 0 );
    // cout << "Finished! Congratulations!" << endl;
    //destroy all windows when you press any key
    //destroyAllWindows();

    // BFMatcher matcher_bf(NORM_HAMMING, true); //使用汉明距离度量二进制描述子，允许交叉验证
    // vector<DMatch> Matches_bf;
    // matcher_bf.match(srcdesc, curdesc, Matches_bf);
    // knnMatches2(srcdesc,curdesc,Matches_bf);

    // assert(Matches_bf.size() > 0);
    // Mat BF_img;
    // drawMatches(imagesrc, srckps, imagecur, curkps, Matches_bf, BF_img);
    // cv::resize(BF_img, BF_img, Size(2*imagesrc.cols, imagesrc.rows));
    // // cout << Matches_bf.size() << endl;
    // putText(BF_img, "Brute Force Matches", Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
    // // imshow("Brute Force Matches", BF_img);
   
    // // // 数据格式预处理
    // vector<Point2f> X;
    // vector<Point2f> Y;
    // X.clear();
    // Y.clear();

 	// // -------------开始代码-------------
    // // 将Matches_bf里的匹配点对分别放到X,Y 向量里，约5行代码，参考OpenCV DMatch类

    // for(int i=0;i<Matches_bf.size();i++){
    //     int index1 = Matches_bf.at(i).queryIdx;
	// 	int index2 = Matches_bf.at(i).trainIdx;
    //     X.push_back(srckps.at(index1).pt);
    //     Y.push_back(curkps.at(index2).pt);
    // }


    // // -------------结束代码-------------

    // // 调用VFC主函数
    // // t = (double)getTickCount();
    // VFC myvfc;
    // myvfc.setData(X, Y);
    // myvfc.optimize();
    // vector<int> matchIdx = myvfc.obtainCorrectMatch();
    // // t = 1000 * ((double)getTickCount() - t) / getTickFrequency();

    // // 筛选正确的匹配
    // std::vector< DMatch > Matches_VFC;
    // for (unsigned int i = 0; i < matchIdx.size(); i++) {
    // int idx = matchIdx[i];
    //     Matches_VFC.push_back(Matches_bf[idx]);
    // }

    // cout << "# Refine Matches (after VFC): " << Matches_VFC.size() << "/" << Matches_bf.size() << ", Times ="<<t<<" ms " << endl;

    // 绘制筛选结果
    // Mat imageVFC;
    // cv::drawMatches(imagesrc, srckps, imagecur, curkps, Matches_VFC, imageVFC);
    // cv::resize(imageVFC, imageVFC, Size(2*imagesrc.cols, imagesrc.rows));
    // putText(imageVFC, "VFC Matches", Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
    // imshow("VFC Matches", imageVFC);
    // cout << Matches_VFC.size() << endl;
    // waitKey(0);




    //KNN匹配
    // cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    // // 设置 k 值
    // int k = 2; // 每个特征点最多匹配 k 个特征点

    // // 进行特征点匹配
    // std::vector<std::vector<cv::DMatch>> knnMatches;
    // matcher->knnMatch(srcdesc, curdesc, knnMatches, k);

    // // 可选：根据匹配程度进行筛选
    // float ratioThreshold = 0.6; // 设置筛选阈值
    // std::vector<cv::DMatch> filteredMatches;
    // for (const std::vector<cv::DMatch>& knnMatch : knnMatches) {
    //     if (knnMatch.size() >= 2) {
    //         float distanceRatio = knnMatch[0].distance / knnMatch[1].distance;
    //         if (distanceRatio < ratioThreshold) {
    //             filteredMatches.push_back(knnMatch[0]);
    //         }
    //     }
    // }

    // // 绘制匹配结果
    // cv::Mat matchImage;
    // cv::drawMatches(imagesrc, srckps, imagecur, curkps, filteredMatches, matchImage);
    //  cout << filteredMatches.size() << endl;
    // cv::imshow("Matches", matchImage);
    // cv::waitKey(0);

