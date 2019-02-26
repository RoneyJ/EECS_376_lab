//find_filter_and_segment.cpp
// segment objects from a point-cloud
// steps: 
//  *convert pointcloud to torso-lift frame, so z-values are measured vertically
//  *filter the points to retain only points within a "box" above the table
//  *convert the filtered points to corresponding pixels in an OpenCV matrix
//  *perform connected-component analysis on 2D image
//  *using result in 

// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn Feb 2019

#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl_ros/transforms.h>

//point-cloud library headers; likely don't need all these
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl_utils/pcl_utils.h>
#include <xform_utils/xform_utils.h>

//headers for using OpenCV functions
#include <opencv2/core/utility.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;
using namespace cv;


//magic numbers for filtering pointcloud points:
//specify the min and max values, x,y, znd z, for points to retain...
// as expressed in the robot's torso frame (torso_lift_link)
const float MIN_X = 0.4; //include points starting 0.4m in front of robot
const float MAX_X = 0.9; //include points out to 0.9m in front of robot
const float MIN_Y = -0.5; //include points starting -0.5m to left of robot
const float MAX_Y = 0.5; //include points up to 0.5m to right of robot
const float MIN_Z = -0.05; //2cm above the table top
const float MAX_Z = 0.05; //consider points up to 0.12m above table top

const float MID_X_PIXEL = 199;
const float MID_Y_PIXEL = 99;

//choose, e.g., resolution of 5mm, so 100x200 image for 0.5m x 1.0m pointcloud
// adjustable--> image  size
// try 400 pix/m...works fine, i.e. 2.5mm resolution
const float PIXELS_PER_METER = 400.0; //200.0;

const int Nu = (int) ((MAX_X - MIN_X) * PIXELS_PER_METER);
const int Nv = (int) ((MAX_Y - MIN_Y) * PIXELS_PER_METER);

Mat_<uchar> bw_img(Nu, Nv);
Mat_<int>labelImage(Nu, Nv);
Mat dst(bw_img.size(), CV_8UC3);

//Global Variables for Object Centroids
vector<float> x_centroids;
vector<float> y_centroids;
//Global Variable for object orientatinos about z-axis
vector<float> orientations;
//Global Variable matrix to hold quaternions
vector<geometry_msgs::Quaternion> vec_of_quat; 
//Major axis and centroids of object for use by functions
Eigen::Vector3f major_axis_,centroid_,plane_normal;

XformUtils *g_xform_ptr;

void find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector3f box_pt_min,
        Eigen::Vector3f box_pt_max, vector<int> &indices) {
    int npts = input_cloud_ptr->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    cout << "box min: " << box_pt_min.transpose() << endl;
    cout << "box max: " << box_pt_max.transpose() << endl;
    for (int i = 0; i < npts; ++i) {
        pt = input_cloud_ptr->points[i].getVector3fMap();

        //check if in the box:
        if ((pt[0] > box_pt_min[0])&&(pt[0] < box_pt_max[0])&&(pt[1] > box_pt_min[1])&&(pt[1] < box_pt_max[1])&&(pt[2] > box_pt_min[2])&&(pt[2] < box_pt_max[2])) {
            //passed box-crop test; include this point
            //ROS_INFO("point passes test");
            //cout<<"pt passed test: "<<pt.transpose()<<endl;
            indices.push_back(i);
        }
    }
    int n_extracted = indices.size();
    cout << " number of points within box = " << n_extracted << endl;
}

//This fnc was used to confirm the table height relative to torso frame;
// It is no longer needed, since the table height, once found, can be hard coded
// i.e., 7cm below the torso-frame origin

double find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double z_min, double z_max, double dz) {
    vector<int> indices;
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object    
    pass.setInputCloud(input_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    double z_table = 0.0;
    int npts_max = 0;
    int npts;
    pcl::PointXYZRGB point;
    int ans;
    for (double z = z_min; z < z_max; z += dz) {
        pass.setFilterLimits(z, z + dz);
        pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
        npts = indices.size();

        if (npts > 0) ROS_INFO("z=%f; npts = %d", z, npts);
        if (npts > npts_max) {
            npts_max = npts;
            z_table = z + 0.5 * dz;
        }
    }
    ROS_INFO("max pts %d at height z= %f", npts_max, z_table);
    return z_table;
}



//use this for Fetch simu:
//assuming head is tilted down 1.0rad, this is the head-camera frame
// with respect to the torso_lift_link frame, expressed as an Eigen::Affine

Eigen::Affine3f compute_affine_cam_wrt_torso_lift_link(void) {
    //pose of cam w/rt floor
    Eigen::Affine3f affine_cam_wrt_torso;
    Eigen::Matrix3f R_cam;
    Eigen::Vector3f O_cam;
    O_cam << 0.244, 0.02, 0.627;
    affine_cam_wrt_torso.translation() = O_cam;
    Eigen::Vector3f nvec, tvec, bvec;
    //magic numbers, as determined by rosrun tf tf_echo torso_lift_link head_camera_rgb_optical_frame
    // when running robot in Gazebo with headcam tilted down 1.0rad
    nvec << 0, -1, 0;
    tvec << -0.8442, 0, -0.5377;
    bvec << 0.5377, 0, -0.8442;
    R_cam.col(0) = nvec;
    R_cam.col(1) = tvec;
    R_cam.col(2) = bvec;
    affine_cam_wrt_torso.linear() = R_cam;
    return affine_cam_wrt_torso;
}

void find_orientation(Eigen::MatrixXf points_mat, float &orientation, geometry_msgs::Quaternion &quaternion) {
    //ROS_INFO("starting identification of plane from data: ");
    int npts = points_mat.cols(); // number of points = number of columns in matrix; check the size
    
    // first compute the centroid of the data:
    //Eigen::Vector3f centroid; // make this member var, centroid_
    centroid_ = Eigen::MatrixXf::Zero(3, 1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    
    //centroid = compute_centroid(points_mat);
     for (int ipt = 0; ipt < npts; ipt++) {
        centroid_ += points_mat.col(ipt); //add all the column vectors together
    }
    centroid_ /= npts; //divide by the number of points to get the centroid    
    cout<<"centroid: "<<centroid_.transpose()<<endl;


    // subtract this centroid from all points in points_mat:
    Eigen::MatrixXf points_offset_mat = points_mat;
    for (int ipt = 0; ipt < npts; ipt++) {
        points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid_;
    }
    //compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3f CoVar;
    CoVar = points_offset_mat * (points_offset_mat.transpose()); //3xN matrix times Nx3 matrix is 3x3
    //cout<<"covariance: "<<endl;
    //cout<<CoVar<<endl;

    // here is a more complex object: a solver for eigenvalues/eigenvectors;
    // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
    Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

    Eigen::VectorXf evals; //we'll extract the eigenvalues to here
    //cout<<"size of evals: "<<es3d.eigenvalues().size()<<endl;
    //cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<endl;
    //cout << "The eigenvalues of CoVar are:" << endl << es3d.eigenvalues().transpose() << endl;
    //cout<<"(these should be real numbers, and one of them should be zero)"<<endl;
    //cout << "The matrix of eigenvectors, V, is:" << endl;
    //cout<< es3d.eigenvectors() << endl << endl;
    //cout<< "(these should be real-valued vectors)"<<endl;
    // in general, the eigenvalues/eigenvectors can be complex numbers
    //however, since our matrix is self-adjoint (symmetric, positive semi-definite), we expect
    // real-valued evals/evecs;  we'll need to strip off the real parts of the solution

    evals = es3f.eigenvalues().real(); // grab just the real parts
    //cout<<"real parts of evals: "<<evals.transpose()<<endl;

    // our solution should correspond to an e-val of zero, which will be the minimum eval
    //  (all other evals for the covariance matrix will be >0)
    // however, the solution does not order the evals, so we'll have to find the one of interest ourselves

    double min_lambda = evals[0]; //initialize the hunt for min eval
    double max_lambda = evals[0]; // and for max eval
    //Eigen::Vector3cf complex_vec; // here is a 3x1 vector of double-precision, complex numbers
    //Eigen::Vector3f evec0, evec1, evec2; //, major_axis; 
    //evec0 = es3f.eigenvectors().col(0).real();
    //evec1 = es3f.eigenvectors().col(1).real();
    //evec2 = es3f.eigenvectors().col(2).real();  
    
    
    //((pt-centroid)*evec)*2 = evec'*points_offset_mat'*points_offset_mat*evec = 
    // = evec'*CoVar*evec = evec'*lambda*evec = lambda
    // min lambda is ideally zero for evec= plane_normal, since points_offset_mat*plane_normal~= 0
    // max lambda is associated with direction of major axis
    
    //sort the evals:
    
    //complex_vec = es3f.eigenvectors().col(0); // here's the first e-vec, corresponding to first e-val
    //cout<<"complex_vec: "<<endl;
    //cout<<complex_vec<<endl;
    plane_normal = es3f.eigenvectors().col(0).real(); //complex_vec.real(); //strip off the real part
    major_axis_ = es3f.eigenvectors().col(0).real(); // starting assumptions
    
    //cout<<"real part: "<<est_plane_normal.transpose()<<endl;
    //est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

    double lambda_test;
    int i_normal = 0;
    int i_major_axis=0;
    //loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
    for (int ivec = 1; ivec < 3; ivec++) {
        lambda_test = evals[ivec];
        if (lambda_test < min_lambda) {
            min_lambda = lambda_test;
            i_normal = ivec; //this index is closer to index of min eval
            plane_normal = es3f.eigenvectors().col(i_normal).real();
        }
        if (lambda_test > max_lambda) {
            max_lambda = lambda_test;
            i_major_axis = ivec; //this index is closer to index of min eval
            major_axis_ = es3f.eigenvectors().col(i_major_axis).real();
        }        
    }

	float x_component = major_axis_(0);
	float y_component = major_axis_(1);
	orientation = atan2(y_component,x_component);

	quaternion = g_xform_ptr->convertPlanarPsi2Quaternion(orientation);
    // at this point, we have the minimum eval in "min_lambda", and the plane normal
    // (corresponding evec) in "est_plane_normal"/
    // these correspond to the ith entry of i_normal
    //cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    //cout<<"corresponding evec (est plane normal): "<<plane_normal.transpose()<<endl;
    //cout<<"max eval is "<<max_lambda<<", corresponding to component "<<i_major_axis<<endl;
    //cout<<"corresponding evec (est major axis): "<<major_axis_.transpose()<<endl;  
    
    //what is the correct sign of the normal?  If the data is with respect to the camera frame,
    // then the camera optical axis is z axis, and thus any points reflected must be from a surface
    // with negative z component of surface normal
    if (plane_normal(2)>0) plane_normal = -plane_normal; // negate, if necessary
    
    //cout<<"correct answer is: "<<normal_vec.transpose()<<endl;
    //cout<<"est plane distance from origin = "<<est_dist<<endl;
    //cout<<"correct answer is: "<<dist<<endl;
    //cout<<endl<<endl;    
    //ROS_INFO("major_axis: %f, %f, %f",major_axis_(0),major_axis_(1),major_axis_(2));
    //ROS_INFO("plane normal: %f, %f, %f",plane_normal(0),plane_normal(1),plane_normal(2));
}

//given a binary image in bw_img, find and label connected regions  (blobs)
// labelImage will contain integer labels with 0= background, 1 = first blob found,
// ...up to nBlobs
// also creates a colored image such that each blob found gets assigned  a
// unique color, suitable for display and visual interpretation,  though this is
// NOT needed by the robot

void blob_color(void) {

    //openCV function to do all the  hard work
    int nLabels = connectedComponents(bw_img, labelImage, 4); //4 vs 8 connected regions

    ROS_INFO("nLabels = %d", nLabels);

    //colorize the regions and display them:
    std::vector<Vec3b> colors(nLabels);
    colors[0] = Vec3b(0, 0, 0); //make the background black
    //assign random color to each region label
    for (int label = 1; label < nLabels; ++label) {
        colors[label] = Vec3b((rand()&255), (rand()&255), (rand()&255));
    }

    //for human consumption: assign colors to regions and display result
    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
            int label = labelImage.at<int>(r, c);
            Vec3b &pixel = dst.at<Vec3b>(r, c);
            pixel = colors[label];
        }
    }
    //display the result in an openCV window
    //imshow("Connected Components", dst); //do this  from "main" instead

    /*
	Parse through dst to find centroids for (nLabels - 1) objects
    */

    int arr_num_pixels[nLabels] = {0};		//stores the number of points contributing to each centroid
    int arr_x_pixel_values[nLabels] = {0};	//stores the x_coordinate of points contributing to each centroid
    int arr_y_pixel_values[nLabels] = {0};	//stores the y_coordinate of points contributing to each centroid

	


    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
	        int label = labelImage.at<int>(r, c);
	        
	        	arr_num_pixels[label]++;
	        	arr_x_pixel_values[label] += c;
	        	arr_y_pixel_values[label] += r;
            
        }
    }

    for(int i = 0; i < nLabels; i++){
    	//calculate centroids
    	float x_centroid = 0.0;
    	float y_centroid = 0.0;

    	y_centroid = (199 - (arr_x_pixel_values[i] / arr_num_pixels[i])) / PIXELS_PER_METER;
    	x_centroid = ((99 - (arr_y_pixel_values[i] / arr_num_pixels[i])) / PIXELS_PER_METER) + ((MAX_X - MIN_X)/2) + MIN_X;

	    x_centroids.push_back(x_centroid);
	    y_centroids.push_back(y_centroid);
	}
	
	Eigen::Vector3f e_pt;
	for(int label = 1; label < nLabels; label++){
		int npts_blob = arr_num_pixels[label];
		Eigen::MatrixXf blob(3, npts_blob);
		int col_num = 0;
		for(int r = 0; r < labelImage.rows; r++){
			for(int c = 0; c < labelImage.cols; c++){
				int label_num = labelImage.at<int>(r,c);
				if(label_num == label){
					e_pt<<c,r,0.0;
					blob.col(col_num) = e_pt;
					col_num++;
				}
			}
		}
		float angle;
		geometry_msgs::Quaternion quat;
		find_orientation(blob, angle, quat);
		orientations.push_back(angle);
		vec_of_quat.push_back(quat);
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    //various pointcloud holders:
    // input, transformed input, downsampled input, box-filtered cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCam_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    vector<int> indices;
    Eigen::Affine3f A_plane_wrt_camera;

    //load a PCD file using pcl::io function; in a robot system, subscribe to pointcloud topic instead    
    string fname;
    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclCam_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclCam_clr_ptr->header.frame_id = "head_camera_rgb_optical_frame";
    ROS_INFO("view frame head_camera_rgb_optical_frame on topic pcd");


    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    ros::Publisher pubBoxFilt = nh.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1);
    sensor_msgs::PointCloud2 ros_cloud, downsampled_cloud, ros_box_filtered_cloud; //here are ROS-compatible messages
    pcl::toROSMsg(*pclCam_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    // this is not really necessary, but it illustrates how to reduce data size, which can be important
    // when more speed is needed.
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclCam_clr_ptr);

    vox.setLeafSize(0.02f, 0.02f, 0.02f); //filter into 2cm voxels
    vox.filter(*downsampled_cloud_ptr);
    cout << "done voxel filtering" << endl;

    cout << "num bytes in original cloud data = " << pclCam_clr_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_cloud_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_cloud_ptr, downsampled_cloud); //convert to ros message for publication and display

    //specify the transform from camera frame to torso frame, expressed as an affine
    //this is desired so points can be filtered according to their z values
    A_plane_wrt_camera = compute_affine_cam_wrt_torso_lift_link();

    //transform the head-camera data into the torso frame
    // result will be in "transformed_cloud_ptr"
    pcl::transformPointCloud(*pclCam_clr_ptr, *transformed_cloud_ptr, A_plane_wrt_camera);

    int npts = transformed_cloud_ptr->points.size();
    cout << "npts transformed = " << npts << endl;

    //don't need this any more; already found the table height
    // but run it just for fun; remove later for better speed
    double table_height = find_table_height(transformed_cloud_ptr, -2.0, 2.0, 0.01);

    //specify opposite corners of a box to box-filter the transformed points
    //magic numbers are at top of this program
    Eigen::Vector3f box_pt_min, box_pt_max;
    box_pt_min << MIN_X, MIN_Y, MIN_Z;
    box_pt_max << MAX_X, MAX_Y, MAX_Z;

    //find which points in the transformed cloud are within the specified box
    //result is in "indices"
    find_indices_box_filtered(transformed_cloud_ptr, box_pt_min, box_pt_max, indices);
    pcl::copyPointCloud(*pclCam_clr_ptr, indices, *box_filtered_cloud_ptr); //extract these pts into new cloud, for display
    pcl::toROSMsg(*box_filtered_cloud_ptr, ros_box_filtered_cloud); //convert to ros message for publication and display
    int npts_cloud = indices.size();

    pubCloud.publish(ros_cloud); //publish original point-cloud, so it is viewable in rviz        
    pubBoxFilt.publish(ros_box_filtered_cloud); //ditto for filtered point cloud
    pubDnSamp.publish(downsampled_cloud); //publish down-sampled original image as well

    //convert point cloud to top-down 2D projection for OpenCV processing
    float x, y, z;
    int index, u, v;
    Eigen::Vector3f cloud_pt;
    bw_img = 0; //initialize image to all black
    labelImage = 0; // ditto for result that  will get populated  with region labels

    //for each pointcloud point, compute which pixel it maps to and find its z value
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        index = indices[ipt];
        cloud_pt = transformed_cloud_ptr->points[index].getVector3fMap();
        z = cloud_pt[2];
        y = cloud_pt[1];
        x = cloud_pt[0];
        //careful: some pointclouds include bad data; test for this
        if ((z == z)&&(x == x)&&(y == y)) { // test for Nan 
            //compute pixel values from metric values
            v = round((y - MIN_Y) * PIXELS_PER_METER);
            u = round((x - MIN_X) * PIXELS_PER_METER);
            //flip/invert these so image makes sense visually
            u = Nu - u;
            v = Nv - v;
            //make sure the computed indices fit within the matrix size:
            if ((u > 0)&&(u < Nu - 1)&&(v > 0)&&(v < Nv - 1)) {
                bw_img(u, v) = 255; //assign this pixel to be white
            }
        }

    }

    //find connected components; 
    //operates on global bw_img and puts region-labeled codes in global Mat "labelImage" 
    blob_color();

    for(int i = 1; i < x_centroids.size(); i++){
    	ROS_INFO("x,y = %.2f, %.2f", x_centroids.at(i), y_centroids.at(i));
    }

	for(int i = 0; i < orientations.size(); i++){
		ROS_INFO("orientation of object %d = %f", i, orientations.at(i));
	}

	for(int i = 0; i < vec_of_quat.size(); i++){
		cout<< vec_of_quat.at(i) << endl;
	}

    //create openCV display windows
    // this is only for debugging; can go away later
    namedWindow("Image", WINDOW_AUTOSIZE);
    namedWindow("Connected Components", WINDOW_AUTOSIZE);
    //colorized regions will be displayed by blob_color();
    //display the original B/W image as well:
    imshow("Image", bw_img); //display the binary image
    imshow("Connected Components", dst);

    waitKey(0); //this is needed to update openCV display windows;
    //it can go away later
	
    return 0;
}
	
