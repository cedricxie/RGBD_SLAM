/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

namespace myslam
{

VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );

    // define local optimizer
    /*typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    optimizer.setAlgorithm ( solver );*/

    // define global optimizer
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,6> > globalBlock;
    //typedef g2o::BlockSolver_6_3 globalBlock;
    //globalBlock::LinearSolverType* globalLinearSolver = new g2o::LinearSolverEigen< globalBlock::PoseMatrixType >();
    //globalLinearSolver->setBlockOrdering( false );
    //globalBlock::LinearSolverType* globalLinearSolver = new g2o::LinearSolverCSparse<globalBlock::PoseMatrixType>(); // 线性方程求解器
    globalBlock::LinearSolverType* globalLinearSolver = new g2o::LinearSolverDense<globalBlock::PoseMatrixType>();
    //globalBlock::LinearSolverType* globalLinearSolver = new g2o::LinearSolverCholmod< globalBlock::PoseMatrixType >();
    globalBlock* global_solver_ptr = new globalBlock ( globalLinearSolver );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* globalSolver = new g2o::OptimizationAlgorithmLevenberg ( global_solver_ptr );
    globalOptimizer_.setAlgorithm( globalSolver );
    // 不要输出调试信息
    globalOptimizer_.setVerbose( true );
}

VisualOdometry::~VisualOdometry()
{

}

bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:
    {
        state_ = OK;
        curr_ = ref_ = frame;
        // extract features from first frame and add them into map
        extractKeyPoints();
        computeDescriptors();
        addKeyFrame();      // the first frame is a key-frame
        break;
    }
    case OK:
    {
        curr_ = frame;
        curr_->T_c_w_ = ref_->T_c_w_;
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        poseEstimationPnP();
        if ( checkEstimatedPose() == true ) // a good estimation
        {
            curr_->T_c_w_ = T_c_w_estimated_;
            optimizeMap();
            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
            }
        }
        else // bad estimation due to various reasons
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            if ( num_lost_ > max_num_lost_ / 4 )
            {
                optimizeMap();
            }
            cout << "num_lost_: " << num_lost_ << endl;
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}

void VisualOdometry::extractKeyPoints()
{
    boost::timer timer;
    orb_->detect ( curr_->color_, keypoints_curr_ );
    cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}

void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
}

void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    // select the candidates in map
    Mat desp_map;
    vector<MapPoint::Ptr> candidate;
    for ( auto& allpoints: map_->map_points_ )
    {
        MapPoint::Ptr& p = allpoints.second;
        // check if p in curr frame image
        if ( curr_->isInFrame(p->pos_) )
        {
            // add to candidate
            p->visible_times_++;
            candidate.push_back( p );
            desp_map.push_back( p->descriptor_ );
        }
    }
    cout<<"# of points in the map / frame: "<<desp_map.size() << " " << descriptors_curr_.size() <<endl;
    matcher_flann_.match ( desp_map, descriptors_curr_, matches );
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    match_3dpts_.clear();
    match_2dkp_index_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            match_3dpts_.push_back( candidate[m.queryIdx] );
            match_2dkp_index_.push_back( m.trainIdx );
        }
    }
    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time: "<<timer.elapsed() <<endl;
}

void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for ( int index:match_2dkp_index_ )
    {
        pts2d.push_back ( keypoints_curr_[index].pt );
    }
    for ( MapPoint::Ptr pt:match_3dpts_ )
    {
        pts3d.push_back( pt->getPositionCV() );
    }

    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
              ref_->camera_->fx_, 0, ref_->camera_->cx_,
              0, ref_->camera_->fy_, ref_->camera_->cy_,
              0,0,1
            );
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 200, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_w_estimated_ = SE3 (
                           SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
                           Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
                       );
    if (num_inliers_ < min_inliers_) {
        return;
    }
    // using bundle adjustment to optimize the pose; moved to initialization
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    //optimizer.clear(); // reset optimizer

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
    ));
    optimizer.addVertex ( pose );

    // edges
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int> ( i,0 );
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId ( i );
        edge->setVertex ( 0, pose );
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );
        edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        // set the inlier map points
        match_3dpts_[index]->matched_times_++;
    }

    optimizer.initializeOptimization();
    optimizer.optimize ( 200 );
    //cout << "optimizing local pose graph, edges: " << optimizer.edges().size() << endl;
    //optimizer.save("./local_result_before.g2o");

    T_c_w_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );

    cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
}

bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if ( d.norm() > 20.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    cout<<"accept with motion: "<<d.norm() <<endl;
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans ) {
        cout<< "IS a key frame" <<endl;
        return true;
    }
    cout<< "NOT a key frame" <<endl;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    // add vertex
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId( curr_->id_ );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    /*v->setEstimate( g2o::SE3Quat(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d( 0,0,0 )
    ) );*/
    /*SE3 T_w_c = curr_->T_c_w_.inverse();
    v->setEstimate( g2o::SE3Quat(
      T_w_c.rotation_matrix(), T_w_c.translation()
    ) );*/
    globalOptimizer_.addVertex(v);

    if (curr_->id_ != ref_->id_) {
      // 边部分
      g2o::EdgeSE3* edge = new g2o::EdgeSE3();
      // 连接此边的两个顶点id
      //edge->setVertex( 0, globalOptimizer_.vertex(ref_->id_ ));
      //edge->setVertex( 1, globalOptimizer_.vertex(curr_->id_ ));
      edge->vertices() [0] = globalOptimizer_.vertex( ref_->id_ );
      edge->vertices() [1] = globalOptimizer_.vertex( curr_->id_ );
      // edge->setRobustKernel( new g2o::RobustKernelHuber() );
      // 信息矩阵
      Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity() * 100;
      //Eigen::Matrix<double, 6, 6> information;
      /*information << 1,  2,  3,  4,  5,  6,
                     7,  8,  9,  10, 11, 12,
                     13, 14, 15, 16, 17, 18,
                     19, 20, 21, 22, 23, 24,
                     25, 26, 27, 28, 29, 30,
                     31, 32, 33, 34, 35, 36;*/
      /*information << 100,  0,  100,  0,   0,   0,
                     0,    0,  0,    100, 0,   0,
                     0,    0,  0,    0,   100, 0,
                     0,    0,  0,    0,   0,   100,
                     0,    0,  0,    0,   0,   0,
                     100,  0,  0,    0,   0,   0;*/
     /*information << 1,    0,  1,    0,   0,   0,
                    0,    0,  0,    1,   0,   0,
                    0,    0,  0,    0,   1,   0,
                    0,    0,  0,    0,   0,   1,
                    0,    0,  0,    0,   0,   0,
                    1,    0,  0,    0,   0,   0;*/
      cout << "information matrix:" << endl;
      // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
      // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
      // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
      //information(0,0) = 100;
      //information(1,1) = 100;
      //information(2,2) = 100;
      //information(3,3) = 100;
      //information(4,4) = 100;
      //information(5,5) = 100;
      // 也可以将角度设大一些，表示对角度的估计更加准确
      edge->setInformation( information);
      cout << edge->information() << endl;
      // 边的估计即是pnp求解之结果
      SE3 T_r_c = ref_->T_c_w_ * curr_->T_c_w_.inverse();
      Eigen::Isometry3d T;
      T.translation() = T_r_c.translation();
      T.linear() = T_r_c.rotation_matrix();
      //edge->setMeasurement( T.inverse() );
      edge->setMeasurement( T );
      // 将此边加入图中
      globalOptimizer_.addEdge(edge);
    }
    else {
        v->setFixed( true ); //第一个顶点固定，不用优化
    }

    if ( map_->keyframes_.empty() )
    {
        // first key-frame, add all 3d points into map
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            double d = curr_->findDepth ( keypoints_curr_[i] );
            if ( d < 0 )
                continue;
            Vector3d p_world = ref_->camera_->pixel2world (
                Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
            );
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            );
            map_->insertMapPoint( map_point );
        }
    }

    map_->insertKeyFrame ( curr_ );
    ref_ = curr_;
}

void VisualOdometry::addMapPoints()
{
    // add the new map points into map
    vector<bool> matched(keypoints_curr_.size(), false);
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
        if ( matched[i] == true )
            continue;
        double d = ref_->findDepth ( keypoints_curr_[i] );
        if ( d<0 )
            continue;
        Vector3d p_world = ref_->camera_->pixel2world (
            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ),
            curr_->T_c_w_, d
        );
        Vector3d n = p_world - ref_->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
        );
        map_->insertMapPoint( map_point );
    }
}

void VisualOdometry::optimizeMap()
{
    cout<<"map points before opti: "<<map_->map_points_.size()<<endl;
    // remove the hardly seen and no visible points
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point
        }
        iter++;
    }

    if ( match_2dkp_index_.size()<1000 || map_->map_points_.size() < 1000)
        addMapPoints();
    if ( map_->map_points_.size() > 1000 )
    {
        // TODO map is too large, remove some one
        map_point_erase_ratio_ += 0.05;
    }
    else
        map_point_erase_ratio_ = 0.1;
    cout<<"map points after opti: "<<map_->map_points_.size()<<endl;
}

double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos( n.transpose()*point->norm_ );
}


}
