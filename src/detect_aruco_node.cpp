// C++ Libraries
#include <iostream>
#include <sstream>
#include <string>

// ROS Libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>	// Image Transport for ROS
#include <cv_bridge/cv_bridge.h>				// OpenCV bridge for ROS

// tf2_ros to broadcast markers poses
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>						// tf2 lib to catch exceptions

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// Classe para subscrever aos topicos de informações da camera RGB "/camera/rgb/camera_info" e ler os parâmetros da camera, criando a matriz de calibração e os coeficientes de distorção.
class CameraInfoListener
{
	// Private Attributes and Methods
	public:
		ros::NodeHandle node_handle_;
		ros::Subscriber camera_info_sub_;
		cv::Mat camera_matrix_;
		cv::Mat distortion_coefficients_;

		bool camera_info_received_ = false;

	// Public Attributes and Methods
	public:
		// Constructor for Class CameraInfoListener
		CameraInfoListener()
		{
			// Subscribe to the camera info topic
			ROS_INFO("\n[CAMERA_INFO]\tSubscribing to the Camera Info Topic...");
			std::string camera_info_topic_name;
			ros::NodeHandle nh_private("~");
			nh_private.param<std::string>("camera_info_topic", camera_info_topic_name, "/camera/rgb/camera_info");
			

			this->camera_info_sub_ = this->node_handle_.subscribe(camera_info_topic_name, 1, &CameraInfoListener::cameraInfoCb, this);
		}

		// Callback function for the camera info subscriber
		void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
		{
			// If camera info hasn't been receive yet, get the camera info from the message and set the camera matrix and distortion coefficients
			if (!camera_info_received_)
			{
				ROS_INFO("\n\n\n-----------------[CAMERA INFO CALLBACK]-----------------\n");
				ROS_INFO("\t[CAMERA_INFO]\tCamera Info Callback Function Called...");
				ROS_INFO("\n[CAMERA_INFO]\tSetting Camera Matrix and Distortion Coefficients...");

				// Set the camera matrix and distortion coefficients
				camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->K.data()).clone();
				distortion_coefficients_ = cv::Mat(1, 5, CV_64F, (void*)msg->D.data()).clone();

				// Print the camera matrix and distortion coefficients
				ROS_INFO_STREAM("\n[CAMERA_INFO]\tCamera Matrix = \n\n" << static_cast<std::ostringstream&>(std::ostringstream() << this->camera_matrix_).str());
				//ROS_INFO_STREAM("\n[CAMERA_INFO]\tCamera Matrix = \n" << this->camera_matrix_);
				ROS_INFO_STREAM("\n[CAMERA_INFO]\tDistortion Coefficients = \n\n" << this->distortion_coefficients_ << std::endl);

				// Set the flag to true to indicate that the camera info has been received
				camera_info_received_ = true;
			}

		}

};



// Classe para detectar marcadores Aruco em um frame do stream da camera RGB, proveniente do topico "/camera/rgb/image_raw" e calcular a pose dos marcadores.
class ArucoPoseEstimator {

	// Private Attributes and Methods
	private:
		ros::Publisher my_publisher_;
		ros::NodeHandle node_handler_;
		image_transport::ImageTransport image_transport_;
		image_transport::Subscriber image_sub_;
		cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_to_use_;

		// tf2_ros broadcaster object
		tf2_ros::TransformBroadcaster tf_broadcaster_;

		// Camera information listener object
		CameraInfoListener camera_info_listener_;

		// Parameters from the parameter server
		std::string image_raw_topic_name_;
		std::string camera_optical_frame_;
		std::string detected_markers_frame_;
		double marker_length_;


	// Public Attributes and Methods
	public:

		// Constructor for Class ArucoPoseEstimator
		ArucoPoseEstimator() : image_transport_(node_handler_)
		{
			// Get parameters from the parameter server
			ros::NodeHandle private_nh = ros::NodeHandle("~");

			// Name of the image topic to get the image from the camera
			private_nh.param<std::string>("image_topic_name", this->image_raw_topic_name_, "/camera/rgb/image_raw");

			// Name of the frames to be used
			private_nh.param<std::string>("camera_optical_frame", this->camera_optical_frame_, "camera_rgb_optical_frame");
			private_nh.param<std::string>("detected_markers_frame", this->detected_markers_frame_, "marker_frame_");

			// Marker length in meters
			private_nh.param<double>("aruco_marker_length", this->marker_length_, 0.176);

			// Initialize the Aruco marker dictionary
			ROS_INFO("\n|\tInitializing the Aruco Dictionary...");
			this->aruco_dictionary_to_use_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

			

			// Publisher
			this->my_publisher_ = this->node_handler_.advertise<geometry_msgs::TransformStamped>(this->detected_markers_frame_ + "topic", 10, true);

			// Subscribe to input video feed and publish output video feed
			ROS_INFO("\n|\tSubscribing to the RGB Image Raw Topic...");
			this->image_sub_ = this->image_transport_.subscribe(this->image_raw_topic_name_, 1,	&ArucoPoseEstimator::getOpenCVImageCb, this);
		}

		/*
		 * Callback function for the image subscriber.
		 * This function is called whenever a new image is published.
		 * It uses cv_bridge to convert ROS image messages to OpenCV images.
		*/
		void getOpenCVImageCb(const sensor_msgs::ImageConstPtr& msg)
		{
			ROS_INFO("\n\n\n-----------------[IMAGE CALLBACK]-----------------\n");
			ROS_INFO("\t|\tImage Callback Function Called...");
			cv_bridge::CvImagePtr cv_image_ptr;	// OpenCV image pointer to store the image
			// try to convert ROS image to OpenCV image and catch any exceptions
			try
			{
				ROS_INFO("\n|\tConverting ROS Image to OpenCV Image...");
				cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			// If any exceptions occur, print the error message in the ROS LOG
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			// Call the function to detect and display Aruco markers
			ROS_INFO("\n|\tCalling function to detect Aruco Markers in the Frame...");
			detectArucoMarkerFromFrame(cv_image_ptr->image);
		}


		void detectArucoMarkerFromFrame(cv::Mat frame)
		{
			std::vector<int> markerIds;
			std::vector<std::vector<cv::Point2f>> markerCorners;

			// Detect the markers in the frame - returns the marker corners and ids
			ROS_INFO("\n|\t\tUsing Aruco Library to Detect Markers...");
			ROS_INFO("\n|\t\tcv::aruco::detectMarkers()...");
			cv::aruco::detectMarkers(frame, this->aruco_dictionary_to_use_, markerCorners, markerIds);

			// Vectors to save the rotation and translation vectors (not matrices) of the detected markers
			std::vector<cv::Vec3d> rvecs, tvecs;

			// If markers detected, draw them on the frame and estimate the marker's pose
			if (not markerIds.empty())
			{
				// Draw the detected markers on the frame
				ROS_INFO("\n|\n|\t\tMarkers Detected... Drawing Markers...");
				cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

				// print marker ids and markerCorners detected.
				ROS_WARN_STREAM("\n|\t\tMarker Corners Detected: " << markerCorners[0]);

				// Estimate the pose of the detected markers
				cv::aruco::estimatePoseSingleMarkers(markerCorners, this->marker_length_, this->camera_info_listener_.camera_matrix_, this->camera_info_listener_.distortion_coefficients_, rvecs, tvecs);


				// DEBUG: print the marker corners, marker length, camera matrix, rvecs and tvecs
				ROS_WARN_STREAM("\n\n\nEstimating Pose of the Detected Markers..." <<
								"\n\nmarkerCorners: \n" <<
								markerCorners[0] <<
								markerCorners[1] <<
								"\n\nMarker Length: " << this->marker_length_ <<
								"\n\nCamera Matrix: \n" <<
									camera_info_listener_.camera_matrix_ <<
								"\n\nDistortion Coefficients: \n" <<
									camera_info_listener_.distortion_coefficients_ <<
								"\n\nrvecs[0]: \n" << rvecs[0] <<
								"\n\ntvecs[0]: \n" << tvecs[0] << "\n");


				// print the distance to each marker detected
				// every marker detected will have its own rvec and tvec
				// and are ordered in the same order as the markerIds returned by the detectMarkers function
				ROS_INFO("\n|\t\tPrinting Marker Information...");
				for (int i = 0; i < markerIds.size(); i++)
				{
					cv::Vec3d rvec = rvecs[i];
					cv::Vec3d tvec = tvecs[i];
					int markerId = markerIds[i];

					// print the distance to the marker detected, showing which marker is being detected
					ROS_INFO("\n|\n|\t\t\t ------ Detected Marker Information ------- ");
					ROS_INFO("\n|\n|\t\t\tMarker ID: %d", markerId);
					ROS_INFO("\n|\t\t\tDistance to marker %d: %f", markerId, tvec[2]);

					ROS_INFO("\n|\n|\t\t\tRotation Vector: [%f, %f, %f]", rvec[0], rvec[1], rvec[2]);
					ROS_INFO("\n|\t\t\tTranslation Vector: [%f, %f, %f]", tvec[0], tvec[1], tvec[2]);
				}

				/*
				 * Transform the rvecs and tvecs to use tf2_ros to broadcast the pose of the detected markers
				*/
				for (int i = 0; i < markerIds.size(); i++)
				{
					try{
						// Create a quaternion from the rotation angles
						tf2::Quaternion quaternion;
						quaternion.setRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);

						// configure the message to be broadcasted
						geometry_msgs::TransformStamped marker2camera;

						marker2camera.header.stamp = ros::Time::now();
						// camera_rgb_optical_frame is the correct frame corresponding to the z of the camera directed to the markers
						marker2camera.header.frame_id = this->camera_optical_frame_; // camera frame of LIMO
						marker2camera.child_frame_id = this->detected_markers_frame_ + std::to_string(markerIds[i]);
						marker2camera.transform.translation.x = tvecs[i][0];
						marker2camera.transform.translation.y = tvecs[i][1];
						marker2camera.transform.translation.z = tvecs[i][2];
						marker2camera.transform.rotation.x = quaternion.x();
						marker2camera.transform.rotation.y = quaternion.y();
						marker2camera.transform.rotation.z = quaternion.z();
						marker2camera.transform.rotation.w = quaternion.w();

						// Broadcast the transform
						this->tf_broadcaster_.sendTransform(marker2camera);

						// Publish the transform in the marker_frame_topic
						this->my_publisher_.publish(marker2camera);

						ROS_INFO("\t\tBroadcasting Marker Pose to TF...");
						ROS_INFO("\t\t\tmarker_frame_%d -> /camera_rgb_optical_frame", markerIds[i]);
					}
					catch (tf2::TransformException &ex) {
            			ROS_WARN("%s", ex.what());
            			ros::Duration(1.0).sleep();
      	  			}
				}
			}
			else
			{
				ROS_WARN("\n|\t\tNo Markers Detected in the Frame...");
			}

			// Display the frame with detected markers
			cv::imshow("Aruco Marker Detector", frame);
			cv::waitKey(3); // Wait for a key press for 3 milliseconds

		}


};



// Main function
int main(int argc, char** argv) {

	ROS_INFO("\n\n----------\nStarting Aruco Marker Detector Node...");

	ros::init(argc, argv, "aruco_detector");
		ROS_INFO("\n -> Initializing the Aruco Pose Estimator...");
		ArucoPoseEstimator aruco_pose_estimator;

    ros::spin();
    return 0;
}

