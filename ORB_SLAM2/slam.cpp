#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <mutex>
#include <ORBVocabulary.h>
#include <KeyFrameDatabase.h>
#include <Tracking.h>
//#include <filesystem>
#include <Converter.h>

using namespace cv;
using namespace pcl;
using namespace std;
using namespace ORB_SLAM2;

mutex stop;


int main(int argc, char* argv[]) {

	string input_video_path, input_depth_path;

	for (int i = 0; i < argc; i++) {
		if (argv[i][0] == '-' && argv[i][1] == 'i') {
			input_video_path = argv[i+1];
		}
		if (argv[i][0] == '-' && argv[i][1] == 'd') {
			input_depth_path = argv[i+1];
		}
	}

	auto reader = VideoCapture(input_video_path);
	auto depth_reader = VideoCapture(input_depth_path);

	namedWindow( "Control Window", WINDOW_AUTOSIZE );
	auto cl = createCLAHE();
	auto orb = ORB::create(1000);
	auto lastImage = Mat(128, 160, CV_8UC3);
	auto image = Mat(128, 160, CV_8UC3);
	auto depth = Mat(128, 160, CV_8UC1);
	auto lbm = cv::StereoBM::create(32, 21);
	auto rbm = cv::StereoBM::create(32, 21);

	boost::shared_ptr<PointCloud<PointXYZRGB>> pointcloud (new PointCloud<PointXYZRGB>);
	pointcloud->width = static_cast<uint32_t>(160);
	pointcloud->height = static_cast<uint32_t>(128);
	pointcloud->is_dense = false;
	visualization::PCLVisualizer viewer("3D Viewer");
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(pointcloud);
	viewer.addPointCloud<PointXYZRGB>(pointcloud,rgb, "our");

	//Init
	const string vocabularyPath = "voc.txt";
	const string settings = "settings.yaml";
	auto vocabulary = make_shared<ORBVocabulary>();
	vocabulary->loadFromTextFile(vocabularyPath);
	auto keyFrameDatabase = new KeyFrameDatabase(*vocabulary);
	auto map = new Map();
	auto tracker = new Tracking(vocabulary.get(), map, keyFrameDatabase, System::RGBD);
	auto localMapping = new LocalMapping(map, true);
	auto localMappingThread = new thread(&LocalMapping::Run,localMapping);
	tracker->SetLocalMapper(localMapping);
	localMapping->SetTracker(tracker);
	
	//Initialize the Loop Closing thread and launch
	auto loopCloser = new LoopClosing(map, keyFrameDatabase, vocabulary.get(), true);
	auto loopClosingThread = new thread(&LoopClosing::Run, loopCloser);
	tracker->SetLoopClosing(loopCloser);
	localMapping->SetLoopCloser(loopCloser);
	loopCloser->SetTracker(tracker);
	loopCloser->SetLocalMapper(localMapping);

	auto t = chrono::system_clock::now();
	int frame = 0;
	while (true) {
		frame++;
		image = Mat();
		bool status = reader.read(image);
		if (!status) break;
		
		depth = Mat();		
		status = depth_reader.read(depth);
		if (!status) break;
		
		auto im = image.clone();
		cvtColor(im,im, COLOR_RGB2GRAY);
		cl->apply(im,im);
		imshow("Control Window", im);

		
		//PerFrame
		double dt = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - t).count()/100;
		//Mat Tcw = mpTracker->GrabImageMonocular(im,dt); // note bw!
		tracker->GrabImageRGBD(im,depth,dt); // note bw+d!
		auto mTrackingState = tracker->mState;
		auto mTrackedMapPoints = tracker->mCurrentFrame.mvpMapPoints;
		auto mTrackedKeyPointsUn = tracker->mCurrentFrame.mvKeysUn;

		//Render
		const auto & vpMPs = map->GetAllMapPoints();
		const auto & vpRefMPs = map->GetReferenceMapPoints();
		set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
		PointXYZRGB point;
		pointcloud->points.clear();

		for(size_t i=0, iend=vpMPs.size(); i<iend;i++) {
			if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i])) {
				continue;
			}
			auto pos = vpMPs[i]->GetWorldPos();
			point.z = pos.at<float>(2);
			point.x = pos.at<float>(0);
			point.y = pos.at<float>(1);
			point.b = 10;
			point.g = 10;
			point.r = 210;

			pointcloud->points.push_back(point);
		}
		for(auto sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++){
			if((*sit)->isBad()) {
				continue;
			}
			auto pos = (*sit)->GetWorldPos();
			point.z = pos.at<float>(2);
			point.x = pos.at<float>(0);
			point.y = pos.at<float>(1);
			point.b = 200;
			point.g = 20;
			point.r = 10;

			pointcloud->points.push_back(point);
		}

		const auto vpKFs = map->GetAllKeyFrames();
		for(size_t k=0; k<vpKFs.size(); k++) {
			auto pKF = vpKFs[k];
			auto Twc = pKF->GetTranslation();
			auto pos = Twc.ptr<float>(0);
			auto rot = Converter::toQuaternion(pKF->GetRotation());
			viewer.addCube(Eigen::Vector3f(pos[0], pos[1], pos[2]),  Eigen::Quaternionf(rot[0],rot[1],rot[2],rot[3]), 0.0001, 0.0001,0.0001,"fr_"+to_string(k));
		}
		cout << pointcloud->points.size() << endl;
		viewer.updatePointCloud(pointcloud, "our");

		lastImage = im.clone();
	}

	pcl::io::savePCDFileASCII ("pointcloud.pcd", pointcloud);



	return 0;
}
