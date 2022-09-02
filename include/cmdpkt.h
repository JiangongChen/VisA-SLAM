/* packet protocol of the information transmitted from server to client
 * including pose return, command to change number of feature points
 */
#ifndef CMDPKT_H
#define CMDPKT_H
using namespace std; 

class CmdPkt{
public: 
    // total length of current packet, unit: bytes
    int total_len_; 
    // a number used to differentiate different commands, 0 for number of feature points
    int cmd_code_; 
    // number of feature points
    int num_fps_; 
    unsigned char* payload; // store the payload

    CmdPkt(unsigned char* buffer, int packet_size) {
        total_len_ = packet_size;
		payload = buffer;

        // first get the code
        unsigned short code_us = (unsigned short)payload[0];
		cmd_code_ = (int) code_us;

        // different commands refers to different format
        if (cmd_code_==0){
            // 0 for number of feature points
            unsigned short fp_us = ((unsigned short) payload[1])*256 + (unsigned short)payload[2];
		    num_fps_ = (int) fp_us;
        }
    }

    // determine the number of feature points in client side
    CmdPkt(int code, int num){
        total_len_ = 3;
        payload = new unsigned char[total_len_];
        payload[0] = (unsigned short)code & 0xff ;
        payload[1] = (unsigned short)num >> 8 ;
        payload[2] = (unsigned short)num & 0xff ;
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

    int getCommandCode(){
        return cmd_code_;
    }
    
    // get the number of feature points
    int getNumFPs(){
        return num_fps_; 
    }
};

#endif