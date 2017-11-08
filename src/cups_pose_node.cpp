#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <algorithm>
#include <fstream>

#define MAX_VALID_DISTANCE 4						//[m]
#define THRESHOLD_CUP_DISTANCE_DETECT 0.05			//[m]
#define THRESHOLD_CUP_N_DETECT 5					//[m]
#define THRESHOLD_DIFF_BETWEEN_CONSECUTIVE_ARC 8	//[ ]
#define THRESHOLD_DIFF_BETWEEN_READINGS 0.1    		//[m]
#define THRESHOLD_FOR_VALID_CLUSTER_LINE_LENGTH 0.2	//[m]
#define THRESHOLD_FOR_VALID_CLUSTER_STD_DEV 0.05	//[m]
#define THRESHOLD_SLOPE_NO_ROTATION_REQUIRED 0.27	//[rad]
#define THRESHOLD_WALL_CONSECUTIVE_ARCS 5			//[ ]
#define THRESHOLD_WALL_SIZE 0.5	//[m]

ros::Publisher *marker_pubPtr;
tf::TransformBroadcaster* BroadPtr;

int estado = 0;

// ############################################################################
// Higher level
void findCup(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

// ############################################################################
// Auxiliary Functions
void fcnConvertPolarCartesian(sensor_msgs::LaserScan laser_msg, std::vector<float> &x, std::vector<float> &y);
float fcnCossineLaw(float a, float b, float angle);
void fcnFilterInvalidValues(sensor_msgs::LaserScan &laser_msg);
void fcnFindClusters(sensor_msgs::LaserScan laser_msg, std::vector<std::pair<int,int> > &clusters_index);
void fcnFindFirstCupFromLeft(sensor_msgs::LaserScan laser_msg, int index_right_extremity_wall, float avg_orthogonal_distance_to_wall, float &x_cup, float &y_cup);
void fcnFindIndexDoor(sensor_msgs::LaserScan laser_msg, int &index_right_extremity);
void fcnFindIndexWall(sensor_msgs::LaserScan laser_msg, int index_right_extremity_door, int &index_right_extremity_wall, float &avg_orthogonal_distance_to_wall);
void fcnGetCorrelationCoefficient(std::vector<float> x, std::vector<float> y, float &R, float &slope, float &intercept, float &line_length, float &std_dev);
void fcnGetCorrelationCoefficientGivenX(std::vector<float> x, std::vector<float> y, float &R, float &slope, float &intercept, float &line_length, float &std_dev);
float fcnMean(std::vector<float> x);
void fcnMedianFilter(sensor_msgs::LaserScan &laser_msg);
void fcnSaveVector(std::string filename, std::vector<float> aux);
float fcnStdDevDiff(std::vector<float> x1,std::vector<float> x2);
float fcnSumSquareDiff(std::vector<float> x1, std::vector<float> x2);

int main(int argc, char **argv)
{
	//Start ROS
	ros::init (argc,argv,"cups_pose_node");
	//Declare node
	ros::NodeHandle n;
	// ROS rate
	ros::Rate loop_rate(20); //Hz

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);    
	// Subscribers
	ros::Subscriber sub_hokuyo = n.subscribe("laserScan", 1, laserCallback);
    tf::TransformBroadcaster broadcaster;	
	BroadPtr = &broadcaster;	
	marker_pubPtr = &marker_pub;

	ros::spin();
	
	return 0;
}

// ############################################################################
// Higher level

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
	findCup(laser_msg);
}

void findCup(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
	static int cnt = 0;
	bool valid_cluster = false;
	int index_right_extremity_door=0, index_right_extremity_wall=0, n_cluster = 0;
	float R=0, intercept=0, slope=0, std_dev=0, line_length=0, avg_orthogonal_distance_to_wall=0, x_cup=0, y_cup=0;
	sensor_msgs::LaserScan laser_msg_copy = *laser_msg;
	std::vector<float> x, y; 
	std::vector<std::pair<int,int> > clusters_index;

	fcnFilterInvalidValues(laser_msg_copy);	
	fcnMedianFilter(laser_msg_copy);
	fcnFindClusters(laser_msg_copy, clusters_index);
	fcnConvertPolarCartesian(laser_msg_copy, x, y);

	// A cluster is valid if it has enough length and bounded standard deviation
	while (!valid_cluster && n_cluster < clusters_index.size())
	{
		fcnGetCorrelationCoefficientGivenX( std::vector<float> (x.begin()+clusters_index[n_cluster].second, x.begin() + clusters_index[n_cluster].second + clusters_index[n_cluster].first -1), std::vector<float> (y.begin()+clusters_index[n_cluster].second, y.begin() + clusters_index[n_cluster].second + clusters_index[n_cluster].first -1), R, slope, intercept, line_length, std_dev );
		cnt++;
		ROS_INFO_STREAM("cnt: " << cnt << "\t\tvalid_cluster: " << valid_cluster << " n_cluster: " << n_cluster);
		
		if (line_length > THRESHOLD_FOR_VALID_CLUSTER_LINE_LENGTH &&std_dev < THRESHOLD_FOR_VALID_CLUSTER_STD_DEV && std::abs(atan(slope)) < THRESHOLD_SLOPE_NO_ROTATION_REQUIRED)
		{
			ROS_INFO_STREAM("CONTINUE AND FIND CUPS!!!");
			valid_cluster = true;
			fcnFindIndexDoor(laser_msg_copy, index_right_extremity_door);
			fcnFindIndexWall(laser_msg_copy, index_right_extremity_door, index_right_extremity_wall, avg_orthogonal_distance_to_wall);
			fcnFindFirstCupFromLeft(laser_msg_copy, index_right_extremity_wall, avg_orthogonal_distance_to_wall, x_cup, y_cup);

			visualization_msgs::Marker cup_marker;
			
			cup_marker.header.frame_id = laser_msg->header.frame_id;
			cup_marker.header.stamp = laser_msg->header.stamp;
			cup_marker.id = 4;
			
			cup_marker.pose.position.x = x_cup;
			cup_marker.pose.position.y = y_cup;
			cup_marker.pose.position.z = 0.1;
					
			cup_marker.pose.orientation.x = 0;
			cup_marker.pose.orientation.y = 0;
			cup_marker.pose.orientation.z = 0;
			cup_marker.pose.orientation.w = 0;
	
			cup_marker.scale.x = 0.05;
			cup_marker.scale.y = 0.5;
			cup_marker.scale.z = 0.2;
			cup_marker.ns = "basic_shapes";
			cup_marker.type = visualization_msgs::Marker::CUBE;
			cup_marker.action = visualization_msgs::Marker::ADD;
			cup_marker.color.r = 0.0f;
			cup_marker.color.g = 1.0f;
			cup_marker.color.b = 1.0f;
			cup_marker.color.a = 1.0;
			cup_marker.lifetime = ros::Duration(1.0);
			
			marker_pubPtr->publish(cup_marker);
			
			// Inicializa a TF
			tf::Transform cup_tf = tf::Transform( tf::Quaternion(0,0,cup_marker.pose.orientation.z,cup_marker.pose.orientation.w), tf::Vector3(cup_marker.pose.position.x, cup_marker.pose.position.y, cup_marker.pose.position.z) );
				// Publica a transformação e atualiza a posição do mapa em relação à odometria		
			BroadPtr->sendTransform( tf::StampedTransform(cup_tf,laser_msg->header.stamp,laser_msg->header.frame_id, "porta"));
		}
		else if (line_length > THRESHOLD_FOR_VALID_CLUSTER_LINE_LENGTH &&std_dev < THRESHOLD_FOR_VALID_CLUSTER_STD_DEV && std::abs(atan(slope))> THRESHOLD_SLOPE_NO_ROTATION_REQUIRED)
		{
			ROS_INFO_STREAM("ROTATE ROBOT!!!");
			valid_cluster = true;
			float rotation_angle = std::abs(atan(slope)); //not sure about sign
		}
		else
		{
			// Continue while and try next cluster...
			n_cluster++;
		}
	}	
}

// ######################################################
// AUXILIARY FUNCTIONS
void fcnConvertPolarCartesian(sensor_msgs::LaserScan laser_msg, std::vector<float> &x, std::vector<float> &y)
{
	x.clear();
	y.clear();
	for (int i = 0; i < (laser_msg.angle_max-laser_msg.angle_min)/laser_msg.angle_increment-1; i++ )
	{
		y.push_back(laser_msg.ranges[i]*cos(laser_msg.angle_max - i*laser_msg.angle_increment));
		x.push_back(laser_msg.ranges[i]*sin(laser_msg.angle_max - i*laser_msg.angle_increment));
	}
}
float fcnCossineLaw(float a, float b, float angle)
{
	return sqrt(pow(a,2) + pow(b,2) - 2*a*b*cos(angle));
}
void fcnFilterInvalidValues(sensor_msgs::LaserScan &laser_msg)
{
	// Transform NAN, INF and values out of range into zero
	for(int i = 0; i < (laser_msg.angle_max-laser_msg.angle_min)/laser_msg.angle_increment-1; i++)
	{
		if(!std::isfinite(laser_msg.ranges[i]) || laser_msg.ranges[i] > MAX_VALID_DISTANCE)
		{
			laser_msg.ranges[i] = 0;
		}
	}
}
void fcnFindClusters(sensor_msgs::LaserScan laser_msg, std::vector<std::pair<int,int> > &clusters_index)
{
	// Divide laser_msg into clusters that have readings close to each other. It aims finding the biggest line in the visible scenario to confirm robot's orientation
	int j = ((laser_msg.angle_max-laser_msg.angle_min)/laser_msg.angle_increment-1) - 2;
	int last_elem = 0, first_elem = 0, diff = 0, k = 0;
	float d1 = 0, d2 = 0;

	while (j > 1)
	{
		last_elem = j;
		k = j;
		d1 = fcnCossineLaw(laser_msg.ranges[k+1], laser_msg.ranges[k],laser_msg.angle_increment);
		d2 = fcnCossineLaw(laser_msg.ranges[k], laser_msg.ranges[k-1],laser_msg.angle_increment);
		while (k > 2 && d2 < THRESHOLD_DIFF_BETWEEN_CONSECUTIVE_ARC*d1 && d2 < THRESHOLD_DIFF_BETWEEN_READINGS)
		{
			k--;
			d1 = fcnCossineLaw(laser_msg.ranges[k+1], laser_msg.ranges[k],laser_msg.angle_increment);
			d2 = fcnCossineLaw(laser_msg.ranges[k], laser_msg.ranges[k-1],laser_msg.angle_increment);
		}		
		first_elem = k;
		j = k-1;
		diff = last_elem-first_elem;
		clusters_index.push_back( std::make_pair(diff, first_elem) );
	}	
	// Sort cluster_index in descending order based on number of readings inside the same cluster
	std::sort(clusters_index.rbegin(), clusters_index.rend());
	/*
	for (int i=0; i<clusters_index.size(); i++)
    {
        ROS_INFO_STREAM(i << ": " << clusters_index[i].first << " " << clusters_index[i].second << std::endl);
	}*/
}
void fcnFindIndexDoor(sensor_msgs::LaserScan laser_msg, int &index_right_extremity)
{
	// Find the index of the right extremity of the door
	float laser_deriv[(int) ((laser_msg.angle_max-laser_msg.angle_min)/laser_msg.angle_increment)];
    float max_deriv[] = {0,0};
    float min_deriv[] = {0,0};
    
    for(int i = 0; i < (laser_msg.angle_max-laser_msg.angle_min)/laser_msg.angle_increment-1; i++)
    {
        laser_deriv[i] = laser_msg.ranges[i+1]-laser_msg.ranges[i];
        if(laser_deriv[i]>max_deriv[0])
        {
            max_deriv[0] = laser_deriv[i];
            max_deriv[1] = i;
        }
        else if(laser_deriv[i]<min_deriv[0])
        {
            min_deriv[0] = laser_deriv[i];
            min_deriv[1] = i;
        }
    }    
    float distSqrd, pA[2], pB[2];
    pA[0] = laser_msg.ranges[max_deriv[1]] * cos( laser_msg.angle_min + laser_msg.angle_increment*max_deriv[1] );
    pA[1] = laser_msg.ranges[max_deriv[1]] * sin( laser_msg.angle_min + laser_msg.angle_increment*max_deriv[1] );
    pB[0] = laser_msg.ranges[min_deriv[1]+1] * cos( laser_msg.angle_min + laser_msg.angle_increment*(min_deriv[1]+1) );
    pB[1] = laser_msg.ranges[min_deriv[1]+1] * sin( laser_msg.angle_min + laser_msg.angle_increment*(min_deriv[1]+1) );
    distSqrd = (pA[0]-pB[0])*(pA[0]-pB[0]) + (pA[1]-pB[1])*(pA[1]-pB[1]);	
	if(distSqrd>0.25 && distSqrd<0.28 && max_deriv[0]>1.4 && min_deriv[0]<-1.4)
	{
		index_right_extremity = max_deriv[1]; //Check if it's index_left_extremity...
	}
	ROS_INFO_STREAM("fcnFindIndexDoor...index_right_extremity_door: " << index_right_extremity);
}
void fcnFindIndexWall(sensor_msgs::LaserScan laser_msg, int index_right_extremity_door, int &index_right_extremity_wall, float &avg_orthogonal_distance_to_wall)
{
	// Find the index of the right extremity of the wall right after door
	index_right_extremity_wall = 0;
	int i = index_right_extremity_door+1;
	float d, d1, d2;
	std::vector<float> x,y;
	while (i < (laser_msg.angle_max-laser_msg.angle_min)/laser_msg.angle_increment-1 && index_right_extremity_wall == 0) 
	{
		d = fcnCossineLaw(laser_msg.ranges[index_right_extremity_door], laser_msg.ranges[i],laser_msg.angle_increment);
		d1 = fcnCossineLaw(laser_msg.ranges[i-1], laser_msg.ranges[i],laser_msg.angle_increment);
		d2 = fcnCossineLaw(laser_msg.ranges[i], laser_msg.ranges[i+1],laser_msg.angle_increment);
		if (d2 > THRESHOLD_WALL_CONSECUTIVE_ARCS*d1 || d > 1.1*THRESHOLD_WALL_SIZE)
		{
			index_right_extremity_wall = i;
		}
		i++;
	}
	fcnConvertPolarCartesian(laser_msg, x, y);
	avg_orthogonal_distance_to_wall = fcnMean(std::vector<float> (y.begin()+index_right_extremity_door, y.begin()+index_right_extremity_wall));
	ROS_INFO_STREAM("fcnFindIndexWall... index_extremity_wall: " << index_right_extremity_wall << "\tavg_orthogonal_distance_to_wall: " << avg_orthogonal_distance_to_wall); 
}
void fcnFindFirstCupFromLeft(sensor_msgs::LaserScan laser_msg, int index_right_extremity_wall, float avg_orthogonal_distance_to_wall, float &x_cup, float &y_cup)
{
	//Find first cup starting from left. It may be possible to tune THRESHOLD_CUP_DISTANCE_DETECT to detect only the first row of of cups
	bool cup_detected = false;
	int n_detects=0;
	int i = index_right_extremity_wall;
	float x_cup_start=0, y_cup_start=0;
	std::vector<float> x,y;
	fcnConvertPolarCartesian(laser_msg, x, y);

	while(!cup_detected && i < (laser_msg.angle_max-laser_msg.angle_min)/laser_msg.angle_increment-1)
	{
		if(std::abs(avg_orthogonal_distance_to_wall - y[i]) < THRESHOLD_CUP_DISTANCE_DETECT)
		{
			if(n_detects == 0)
			{
				x_cup_start = x[i];
				y_cup_start = y[i];
			}
			else
			{
				x_cup = x[i];
				y_cup = y[i];
			}
			n_detects++;			
		}
		else
		{
			n_detects = 0;
		}
		if(n_detects >= THRESHOLD_CUP_N_DETECT)
		{
			cup_detected = true;
			x_cup = 0.5*(x_cup+x_cup_start);
			y_cup = 0.5*(y_cup+y_cup_start);
		} 
		i++;
	}
	ROS_INFO_STREAM("fcnFindCupFromLeft... x_cup: " << x_cup << "\ty_cup: " << y_cup);
}
void fcnGetCorrelationCoefficientGivenX(std::vector<float> x, std::vector<float> y, float &R, float &slope, float &intercept, float &line_length, float &std_dev)
{
	float Sxy = 0, Sx = 0, Sy = 0, Sxx = 0, Syy = 0, meanx = 0, meany = 0;
	float numR=0, denR=0;
	int n = x.size(), n_valid = 0;    
	std::vector<float> y_fitted;
	
	for (int i = 0; i< n;i++)
	{
		if(x[i]!=0)
		{
			Sxy = Sxy + x[i]*y[i];
			Sx = Sx + x[i];
			Sy = Sy + y[i];
			Sxx = Sxx + pow(x[i],2);
			Syy = Syy + pow(y[i],2);
			n_valid++;
			meanx += x[i];
			meany += y[i];
		}
	}	
	
	meanx /= n_valid;
	meany /= n_valid;
	
	slope = (n_valid*Sxy - Sx*Sy)/(n_valid*Sxx - Sx*Sx);
	intercept = (Sy*Sxx - Sx*Sxy)/(n_valid*Sxx - Sx*Sx);

	numR = (n_valid*Sxy - Sx*Sy);
	denR = sqrt(n_valid*Sxx - pow(Sx,2))*sqrt(n_valid*Syy - pow(Sy,2));
	R = numR/denR;  
	for(int i = 0; i < y.size(); i++ )
	{
		y_fitted.push_back(slope*(y[i]) + intercept);
	}
	std_dev = fcnStdDevDiff(y, y_fitted);   
	line_length = sqrt(pow(x[0]-x.back(),2) + pow(y[0]-y.back(),2));

	fcnSaveVector("x.txt", x);
	fcnSaveVector("y.txt", y_fitted);
	ROS_INFO_STREAM("slope: " << slope << "\tintercept: " << intercept << "\tsumSquareDiff: " << std_dev << "\tline_length: " << line_length);

}
void fcnGetCorrelationCoefficient(std::vector<float> y, std::vector<float> x, float &R, float &slope, float &intercept, float &line_length, float &std_dev)
{
	// x = slope*y + intercept
	float Sxy = 0, Sx = 0, Sy = 0, Sxx = 0, Syy = 0, meanx = 0, meany = 0;
	float numR=0, denR=0;
	int n = x.size(), n_valid = 0;   
	std::vector<float> x_fitted;
	
	for (int i = 0; i< n;i++)
	{
		if(x[i]!=0)
		{
			Sxy = Sxy + x[i]*y[i];
			Sx = Sx + x[i];
			Sy = Sy + y[i];
			Sxx = Sxx + pow(x[i],2);
			Syy = Syy + pow(y[i],2);
			n_valid++;
			meanx += x[i];
			meany += y[i];
		}
	}	
	
	meanx /= n_valid;
	meany /= n_valid;
	
	slope = (Sxy - meanx*Sy)/(Syy - meany*Sy);
	intercept = meanx - slope*meany;	

	numR = (n_valid*Sxy - Sx*Sy);
	denR = sqrt(n_valid*Sxx - pow(Sx,2))*sqrt(n_valid*Syy - pow(Sy,2));
	R = numR/denR;  
	for(int i = 0; i < x.size(); i++ )
	{
		x_fitted.push_back(slope*(x[i]) + intercept);
	}
	std_dev = fcnStdDevDiff(x, x_fitted);   
	
	fcnSaveVector("x.txt", x_fitted);
	fcnSaveVector("y.txt", y);
}
float fcnMean(std::vector<float> x)
{
		int n = x.size();
		float aux;
		for (int i = 0; i < n; ++i)
		{
			aux += x[i];	
		}
		return aux/n;
}
void fcnMedianFilter(sensor_msgs::LaserScan &laser_msg)
{
	sensor_msgs::LaserScan laser_copy = laser_msg;
	std::vector<float> lidar_vec_cluster (3);	

	for (int i = 1; i < (laser_msg.angle_max-laser_msg.angle_min)/laser_msg.angle_increment-1; i++)
	{
		lidar_vec_cluster[0] = laser_copy.ranges[i-1];
		lidar_vec_cluster[1] = laser_copy.ranges[i];
		lidar_vec_cluster[2] = laser_copy.ranges[i+1];
		std::sort(lidar_vec_cluster.begin(), lidar_vec_cluster.end());
		laser_msg.ranges[i] = lidar_vec_cluster[1];
	}
}
void fcnSaveVector(std::string filename, std::vector<float> aux)
{
	std::fstream savefile;
	savefile.open(filename.c_str(), std::ios_base::out);
	for (int i = 0; i < aux.size(); i++)
	{
		savefile << aux[i] << "|";
	}
	savefile << std::endl;
	savefile.close();
}
float fcnStdDevDiff(std::vector<float> x1,std::vector<float> x2)
{
	std::vector<float> x;
	for (int i = 0; i < x1.size(); i++)
	{
		x.push_back(x1[i]-x2[i]);
	}
	float mean_x=0.0, sum_deviation=0.0;
	int n = x.size();
	mean_x = fcnMean(x);
	for(int i = 0; i < n; ++i)
			sum_deviation+=(x[i]-mean_x)*(x[i]-mean_x);
	return sqrt(sum_deviation/(n-1));   
}	
float fcnSumSquareDiff(std::vector<float> x1, std::vector<float> x2)
{
	// Continue while and try next cluster...
	float sum = 0;
	for (int i = 0; i < x1.size(); i++)
	{
		sum += pow(x1[i]-x2[i],2);
	}
	return sum;
}
