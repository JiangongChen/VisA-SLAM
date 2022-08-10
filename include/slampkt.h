#pragma once
// communication packet between server and client for the edge-SLAM
// each slam packet corresponds to a frame
#ifndef SLAMPKT_H
#define SLAMPKT_H
#include <opencv2/core/core.hpp>
#include <iostream> 

using namespace cv; 
using namespace std; 

class SlamPkt {
public:
	int num_pts_; // number of feature points for this frame, assume no distortion
	int total_len_; // total length of current packet, unit: bytes
	int pt_len_=36; // length of a keypoint, each point use 36 bytes, 2 bytes x, 2 bytes y, plus 32 bytes descriptor
	int header_len_=4; // length of the header, currently, two bytes for frame id, two bytes for ground truth id
	int descriptor_len_ = 32; // length of descriptor for a keypoint, 32 bytes
	int frame_id_; // the frame id
	int ground_truth_id_; // indicates the number of ground truth points during measurements
	unsigned char* payload; // store the payload
	vector<KeyPoint> kps_; // all keypoints, without undistortion
	Mat descriptors_; // descriptors for all keypoints

	SlamPkt(unsigned char* buffer, int packet_size) {
		total_len_ = packet_size;
		payload = buffer;

		// process header
		unsigned short id_us = ((unsigned short) payload[0])*256 + (unsigned short)payload[1];
		frame_id_ = (int) id_us;

		unsigned short gt_id_us = ((unsigned short) payload[2])*256 + (unsigned short)payload[3];
		ground_truth_id_ = (int) gt_id_us;

		num_pts_ = (total_len_-header_len_) / pt_len_;
		descriptors_ = Mat(num_pts_, descriptor_len_, CV_8UC1);
		for (int i = 0; i < num_pts_; i++) {
			unsigned short x = ((unsigned short) payload[i* pt_len_ + header_len_])*256 + (unsigned short)payload[i * pt_len_ + 1 + header_len_];
			unsigned short y = ((unsigned short) payload[i * pt_len_ + 2 + header_len_])*256 + (unsigned short)payload[i * pt_len_ + 3 + header_len_];
			kps_.push_back(KeyPoint(x,y,1));
			for (int j = 0; j < descriptor_len_; j++) {
				descriptors_.at<unsigned char>(i,j) = payload[i * pt_len_ + 4 + j + header_len_];
			} 
		}
	}

	SlamPkt(int id, vector<KeyPoint>& kps, Mat& descriptors, int gt_id) {
		num_pts_ = (int) kps.size(); 
		total_len_ = num_pts_ * pt_len_ + header_len_;
		frame_id_ = id;
		ground_truth_id_ = gt_id;
		kps_ = kps; 
		descriptors_ = descriptors; 
		payload = new unsigned char[total_len_];

		// add header
		payload[0] = (unsigned short)frame_id_ >> 8 ;
		payload[1] = (unsigned short)frame_id_ & 0xff ;
		payload[2] = (unsigned short)ground_truth_id_ >> 8 ;
		payload[3] = (unsigned short)ground_truth_id_ & 0xff ;

		for (int i = 0; i < num_pts_; i++) {
			payload[i * pt_len_ + header_len_] = (unsigned short)kps_[i].pt.x >> 8 ;
			payload[i * pt_len_ + 1 + header_len_] = (unsigned short)kps_[i].pt.x & 0xff;
			payload[i * pt_len_ + 2 + header_len_] = (unsigned short)kps_[i].pt.y >> 8 ;
			payload[i * pt_len_ + 3 + header_len_] = (unsigned short)kps_[i].pt.y & 0xff;
			for (int j = 0; j < descriptor_len_; j++) {
				payload[i * pt_len_ + 4 + j + header_len_] = descriptors_.at<unsigned char>(i, j);
			}
		}
	}

	vector<KeyPoint> getKeyPoints() {
		return kps_; 
	}

	Mat getDescriptors() {
		return descriptors_; 
	}

	unsigned char* getPayload() {
		return payload; 
	}

	unsigned char* getHead(){
		if (total_len_>65536) return nullptr;
		// currently, head only contains the packet size, assume the packet size is lower than 65536
		unsigned char* head = new unsigned char[2];
		head[0] = (unsigned short)total_len_ >> 8 ;
		head[1] = (unsigned short)total_len_ & 0xff;
		return head;
	}

	int getTotalLength(){
		return total_len_;
	}

	int getNumPoints(){
		return num_pts_;
	}

	int getFrameId(){
		return frame_id_;
	}

	int getGroundTruthId(){
		return ground_truth_id_;
	}
}; 
#endif // SLAMPKT_H
