// /WHOLEARCHIVE:tf_cc.lib (in case of NoOp binary error)

#include <iostream>
#define NOMINMAX

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tensorflow/core/public/session.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/util/command_line_flags.h"
#include <tensorflow/core/protobuf/meta_graph.pb.h>

#include <time.h>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <atomic>
#include <fstream>

using tensorflow::Flag;
using tensorflow::Tensor;
using tensorflow::Status;
using tensorflow::string;
using tensorflow::int32;

using namespace tensorflow;
using namespace std;
using namespace cv;

std::mutex _lock;
std::atomic<bool> reading_done(false);
std::atomic<bool> processing_done(false);
std::atomic<bool> need_resize(false);
std::atomic<bool> isXYZsaved(false);

void process_image(Session* session, Mat img, Mat& result) {
	
	//Opening the Image
	int height = 228;
	int width = 304;
	int depth = 3;
	cv::resize(img, img, Size(width, height), 0, 0, CV_INTER_CUBIC);

	//Copying data prom cv::Mat to tensorflow::Tensor
	tensorflow::Tensor keep_prob = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape());
	keep_prob.scalar<float>()() = 1.0;

	tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({ 1, height, width, depth }));
	auto input_tensor_mapped = input_tensor.tensor<float, 4>();

	//copying data from cv::Mat to tensorflow::Tensor
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			Vec3b color = img.at<Vec3b>(y, x);
			input_tensor_mapped(0, y, x, 0) = color[2];
			input_tensor_mapped(0, y, x, 1) = color[1];
			input_tensor_mapped(0, y, x, 2) = color[0];
		}
	}

	//Running model
	std::vector<tensorflow::Tensor> output;
	string input_layer = "Placeholder:0";
	string output_layer = "ConvPred/ConvPred:0";


	Status status = session->Run({ { input_layer, input_tensor } }, { output_layer }, {}, &output);
	if (!status.ok()) {
		cerr << "Error running the model: " + status.ToString();
	}


	//copying output tensor to cv::Mat
	auto output_tensor_mapped = output[0].tensor<float, 4>();
	cv::Mat raw_output_image = cv::Mat(128, 160, CV_32F);
	for (int y = 0; y < 128; y++) {
		for (int x = 0; x < 160; x++) {
			raw_output_image.at<float>(y, x) = output_tensor_mapped(0, y, x, 0);
		}
	}


	if(!isXYZsaved) {
		int centerY = 64;
		int centerX = 80;
		std::ofstream outfile;
		outfile.open("cloud.xyz", std::ios_base::out);
		
		float focal_length = 525.0f;
		for (int y = 0; y < 128; y++) {
			for (int x = 0; x < 160; x++) {
				
				float depth = raw_output_image.at<float>(y, x);
				float ptz = depth*0.001f;
				float ptx = static_cast<float>(x-centerX) * ptz / focal_length;
				float pty = static_cast<float>(y-centerY) * ptz / focal_length;
				outfile << std::to_string(ptx) + " " + std::to_string(pty) + " " + std::to_string(ptz) + "\n";
			}		
		}

		outfile.close();		
		isXYZsaved = true;	
	}

	//finding min and max depth values
	double min;
	double max;
	cv::minMaxIdx(raw_output_image, &min, &max);
	cv::Mat adjMap;

	//expanding range to 0..255
	raw_output_image.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

	//It converts grayscale image into a tone-mapped one
	cv::Mat ColorsMap;
	applyColorMap(adjMap, ColorsMap, cv::COLORMAP_BONE);

	if (need_resize) {
		cv::resize(ColorsMap, ColorsMap, Size(625, 500), 0, 0, CV_INTER_CUBIC);
	}
	
	result = ColorsMap.clone();
}

void readingThread(VideoCapture& reader, queue<shared_ptr<vector<Mat>>>& processingQueue) {
	
	Mat img;
	vector<Mat> imageVector;
	double totalFrames = reader.get(CV_CAP_PROP_FRAME_COUNT);
	
	while (reader.read(img)) {
		imageVector.push_back(img.clone());
		double position = reader.get(CV_CAP_PROP_POS_FRAMES) - 1;
		
		if (imageVector.size() == 20 || (position == (totalFrames - 1))) {

			shared_ptr<vector<Mat>> ptr = make_shared<vector<Mat>>();
			for (auto & im : imageVector) {
				ptr->push_back(im.clone());
				im = Mat();
			}
			
			_lock.lock();
			processingQueue.push(ptr);
			_lock.unlock();

			imageVector.clear();
		}
	}
	cout << "reading_done" << endl;
	reading_done = true;
}

void processingThread(Session* session, queue<shared_ptr<vector<Mat>>>& processingQueue, queue<shared_ptr<vector<Mat>>>& writingQueue) {
	
	vector<Mat> writingVector;

	while (!reading_done) {
		//sleeping while queue is empty
		while (processingQueue.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		//processing until queue has elements
		while (!processingQueue.empty()) {
			_lock.lock();
			shared_ptr<vector<Mat>> ptr = processingQueue.front();
			cout << "taking, current processing queue size: " << processingQueue.size() * 20 << " frames" << endl;
			processingQueue.pop();
			_lock.unlock();
			
			if (!ptr) {
				cout << "bad data!" << endl;
				continue;
			}
			vector<Mat> &imageVector = *ptr;

			for (int i = 0; i < imageVector.size(); i++) {
				Mat ColorsMap;
				Mat img = imageVector[i].clone();
				
				process_image(session, imageVector[i], ColorsMap);
				writingVector.push_back(ColorsMap.clone());

				if (writingVector.size() == 20 || (i == (imageVector.size() - 1))) {
					shared_ptr<vector<Mat>> ptr = make_shared<vector<Mat>>();
					for (auto & im : writingVector) {
						ptr->push_back(im.clone());
						im = Mat();
					}

					_lock.lock();
					writingQueue.push(ptr);
					_lock.unlock();

					writingVector.clear();
				}
			}
		}
	}

	//when reading is done we will process images left
	while (!processingQueue.empty()) {
		_lock.lock();
		shared_ptr<vector<Mat>> ptr = processingQueue.front();
		cout << "taking, current processing queue size: " << processingQueue.size()*20 << " frames" << endl;
		processingQueue.pop();
		_lock.unlock();

		vector<Mat> &imageVector = *ptr;

		for (int i = 0; i < imageVector.size(); i++) {
			Mat ColorsMap;
			process_image(session, imageVector[i], ColorsMap);
			writingVector.push_back(ColorsMap.clone());

			if (writingVector.size() == 20 || (i == (imageVector.size() - 1))) {
				shared_ptr<vector<Mat>> ptr = make_shared<vector<Mat>>();
				for (auto & im : writingVector) {
					ptr->push_back(im.clone());
					im = Mat();
				}

				_lock.lock();
				writingQueue.push(ptr);
				_lock.unlock();

				writingVector.clear();
			}
		}
	}
	cout << "processing_done" << endl;
	processing_done = true;
}

void writingThread(VideoWriter& writer, queue<shared_ptr<vector<Mat>>>& writingQueue) {
	
	while (!processing_done) {
		//sleep while queue is empty
		while (writingQueue.empty() && !processing_done) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		//writing until queue has elements
		while (!writingQueue.empty()) {
			_lock.lock();
			shared_ptr<vector<Mat>> ptr = writingQueue.front();
			cout << "writing, current writing queue size: " << writingQueue.size()*20 << " frames" << endl;
			writingQueue.pop();
			_lock.unlock();

			vector<Mat> &imageVector = *ptr;

			for (int i = 0; i < imageVector.size(); i++) {
				Mat img = imageVector[i];
				writer.write(img);
			}
		}
	}

	//when processing is done we will write images left
	while (!writingQueue.empty()) {
		_lock.lock();
		shared_ptr<vector<Mat>> ptr = writingQueue.front();
		cout << "writing, current writing queue size: " << writingQueue.size() * 20 << " frames" << endl;
		writingQueue.pop();
		_lock.unlock();

		vector<Mat> &imageVector = *ptr;
		for (int i = 0; i < imageVector.size(); i++) {
			Mat img = imageVector[i];
			writer.write(img);
		}
	}
	
	writer.release();
}

int main(int argc, char* argv[]) {

	cout.setf(ios::fixed);

	// set up your input paths
	const string modelName = "NYU_FCRN.ckpt.meta";
	const string checkpointFilename = "NYU_FCRN.ckpt";
	string input_video_path, output_video_path;

	for (int i = 0; i < argc; i++) {
		if (argv[i][0] == '-' && argv[i][1] == 'i') {
			input_video_path = argv[i+1];
		}
		if (argv[i][0] == '-' && argv[i][1] == 'o') {
			output_video_path = argv[i+1];
		}
		if (argv[i][0] == '-' && argv[i][1] == 'r') {
			need_resize = true;
		}
	}

	auto session = NewSession(SessionOptions());
	if (session == nullptr) {
		throw runtime_error("Could not create Tensorflow session.");
	}

	Status status;

	// Read in the protobuf graph we exported
	MetaGraphDef graph_def;
	status = ReadBinaryProto(Env::Default(), modelName, &graph_def);
	if (!status.ok()) {
		throw runtime_error("Error reading graph definition from " + modelName + ": " + status.ToString());
	}

	// Add the graph to the session
	status = session->Create(graph_def.graph_def());
	if (!status.ok()) {
		throw runtime_error("Error creating graph: " + status.ToString());
	}

	// Read weights from the saved checkpoint
	Tensor checkpointPathTensor(DT_STRING, TensorShape());
	checkpointPathTensor.scalar<std::string>()() = checkpointFilename;
	status = session->Run(
	{ { graph_def.saver_def().filename_tensor_name(), checkpointPathTensor }, },
	{},
	{ graph_def.saver_def().restore_op_name() },
		nullptr);
	if (!status.ok()) {
		throw runtime_error("Error loading checkpoint from " + checkpointFilename + ": " + status.ToString());
	}
	
    //setting up reader
	auto reader = VideoCapture(input_video_path);

	//setting up writer
	auto codec = CV_FOURCC('X', 'V', 'I', 'D');
	auto fps = 30.0;

	Size output_video_size;
	if (need_resize) {
		output_video_size = Size(625, 500);
	}
	else {
		output_video_size = Size(160, 128);
	}

	auto writer = VideoWriter(output_video_path, codec, fps, output_video_size, true);


    //creating queues
	queue<shared_ptr<vector<Mat>>> processingQueue;
	queue<shared_ptr<vector<Mat>>> writingQueue;

	clock_t tStart = clock();

	//starting threads
	std::thread readingThr(readingThread, ref(reader), ref(processingQueue));
	std::thread processingThr(processingThread, session, ref(processingQueue), ref(writingQueue));
	std::thread writingThr(writingThread, ref(writer), ref(writingQueue));

    //stopping threads
	readingThr.join(); cout << "reading thread has been terminated" << endl;
	processingThr.join(); cout << "processing thread has been terminated" << endl;
	writingThr.join(); cout << "writing thread has been terminated" << endl;

	printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
	return EXIT_SUCCESS;
}
