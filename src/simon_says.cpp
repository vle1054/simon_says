/*
Human Centered Robotics
Final Project
Group 10 - simon_says
Collin Quinn
Logan Goodrich
Vinh Le
Built on code from github.com/shinselrobots/astra_body_tracker
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include "ros/console.h"
#include "geometry_msgs/PoseStamped.h"
#include <ctime>
#include <string>
#include <sstream>
#include <stdlib.h>

//For Orbbec Astra SDK
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

#include <astra_body_tracker/BodyInfo.h>  // Publish custom message
#include <visualization_msgs/Marker.h>

using namespace astra_body_tracker;

#define PI 3.1415;
float TOLERANCE = 30;

std::string hand[3] = {"Left hand", "Right Hand", "Both Hands"};
std::string action[3] = {"Up", "Down", "To the Side"};
std::string says[2] = {"Simon says"," "};
// hand position [L,R]
int state_curr[] = {1,1};
int state_next[] = {1,1};
int rand_hand, rand_act, rand_says;
int simoncmd;
bool actioncom = true;
bool actionissued = false;
int num_players = 0;
int playerNo;
int playerId[20] = {0};
int gameround = 0;
int maxrounds = 5;

clock_t time_start;

void txttospeak(std::string text);
int find_player_no(int body_id);

template <typename T>
std::string ToString(T val){
  char buffer [50];
  sprintf(buffer, "%d",val);
  return buffer;
}

class astra_body_tracker_node
{
public:
  astra_body_tracker_node(std::string name) :
    _name(name)
  {
    ROS_INFO("%s: Initializing", _name.c_str());
    bool initialized = false;

    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("myparm1",myparm1_,"mydefault");

    // Subscribers
    //robot_behavior_state_ = nh_.subscribe("/behavior/cmd", 1, &behavior_logic_node::behaviorStateCB, this);

    // PUBLISHERS
    body_tracking_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("astra_body_tracker/pose", 1); // NOTE: We only provide to POSITION not full pose

    body_tracking_data_pub_ = nh_.advertise<astra_body_tracker::BodyInfo>("astra_body_tracker/data", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("astra_body_tracker/marker", 1);

    ROS_INFO("astra_body_tracker: Advertised Publisher: astra_body_tracker/data, marker");

  }

  ~astra_body_tracker_node()
  {
    ROS_INFO("astra_body_tracker_node shutting down");
  }







void output_bodies(astra_bodyframe_t bodyFrame){
  int i;
  astra_body_list_t bodyList;
  const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
  if (rc != ASTRA_STATUS_SUCCESS){
    printf("Error %d in astra_bodyframe_body_list()\n", rc);
    return;
  }

  for(i = 0; i < bodyList.count; ++i){
    astra_body_t* body = &bodyList.bodies[i];

    // Pixels in the body mask with the same value as bodyId are
    // from the same body.
    astra_body_id_t bodyId = body->id;

    // Tracking status
    // NOT_TRACKING = 0
    // TRACKING_LOST = 1
    // TRACKING_STARTED = 2
    // TRACKING = 3

    astra_body_status_t bodyStatus = body->status;
    if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED){
      playerId[num_players] = bodyId;
      num_players += 1;
      std::string bodytxt = "Player "+ ToString(num_players) + " has joined the game.";
      printf("Body Id: %d Status: Tracking started\n", bodyId);
      txttospeak(bodytxt);
    }
    else if (bodyStatus == ASTRA_BODY_STATUS_TRACKING){
      printf("Body Id: %d Status: Tracking\n", bodyId);
    }
    else if (bodyStatus == ASTRA_BODY_STATUS_LOST){
      playerNo = find_player_no(bodyId);
      std::string bodytxt = "Player "+ ToString(playerNo) + " has left the game.";
      printf("Body %u Status: Tracking lost.\n", bodyId);
      txttospeak(bodytxt);
      time_start = clock();
    }
    else{ // bodyStatus == ASTRA_BODY_STATUS_NOT_TRACKING
      printf("Body Id: %d Status: Not Tracking\n", bodyId);
    }

    const astra_vector3f_t* centerOfMass = &body->centerOfMass;
    const astra_body_tracking_feature_flags_t features = body->features;
    astra_joint_t* joint;  // AstraSDK/include/astra/capi/streams/body_types.h

    const bool jointTrackingEnabled =
    (features & ASTRA_BODY_TRACKING_JOINTS) == ASTRA_BODY_TRACKING_JOINTS;
    const bool handPoseRecognitionEnabled =
    (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

    ///////////////////////////////////////////////////////////////
    // Publish body tracking information, and display joint info for debug

    // Create structures for ROS Publisher data
    geometry_msgs::PoseStamped body_pose;
    astra_body_tracker::BodyInfo_ <astra_body_tracker::BodyInfo> body_info;



    // Basic Pose for person location tracking
    body_pose.header.frame_id = "astra_camera_link"; // "base_link";
    body_pose.header.stamp = ros::Time::now();

    // Skeleton Data for publilshing more detail

    body_info.body_id = bodyId;
    body_info.tracking_status = bodyStatus;


    joint = &body->joints[ASTRA_JOINT_LEFT_ELBOW];
    body_info.joint_position_left_elbow.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
    body_info.joint_position_left_elbow.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
    body_info.joint_position_left_elbow.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

    joint = &body->joints[ASTRA_JOINT_LEFT_HAND];
    body_info.joint_position_left_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
    body_info.joint_position_left_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
    body_info.joint_position_left_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;


    joint = &body->joints[ASTRA_JOINT_RIGHT_ELBOW];
    body_info.joint_position_right_elbow.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
    body_info.joint_position_right_elbow.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
    body_info.joint_position_right_elbow.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

    joint = &body->joints[ASTRA_JOINT_RIGHT_HAND];
    body_info.joint_position_right_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
    body_info.joint_position_right_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
    body_info.joint_position_right_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;


    ////////////////////////////////////////////////////
    // Publish Body Data
    body_tracking_pose_pub_.publish(body_pose); // this is position only!
    body_tracking_data_pub_.publish(body_info);
    ////////////////////////////////////////////////////


    PublishMarker(
    ASTRA_JOINT_RIGHT_HAND, // ID
    body_info.joint_position_right_hand.x,
    body_info.joint_position_right_hand.y,
    body_info.joint_position_right_hand.z,
    1.0, 0.0, 0.0 ); // r,g,b
    PublishMarker(
    ASTRA_JOINT_RIGHT_ELBOW, // ID
    body_info.joint_position_right_elbow.x,
    body_info.joint_position_right_elbow.y,
    body_info.joint_position_right_elbow.z,
    1.0, 0.0, 0.0 ); // r,g,b

    PublishMarker(
    ASTRA_JOINT_LEFT_HAND, // ID
    body_info.joint_position_left_hand.x,
    body_info.joint_position_left_hand.y,
    body_info.joint_position_left_hand.z,
    1.0, 0.0, 0.0 ); // r,g,b
    PublishMarker(
    ASTRA_JOINT_LEFT_ELBOW, // ID
    body_info.joint_position_left_elbow.x,
    body_info.joint_position_left_elbow.y,
    body_info.joint_position_left_elbow.z,
    1.0, 0.0, 0.0 ); // r,g,b

    //m to inch 39.3

    printf("Body ID : %3.0i\n", bodyId);

    float angle_r = atan2(body_info.joint_position_right_hand.z - body_info.joint_position_right_elbow.z,\
    body_info.joint_position_right_hand.y  - body_info.joint_position_right_elbow.y ) * 180/PI;
    float angle_l = atan2(body_info.joint_position_left_hand.z - body_info.joint_position_left_elbow.z, \
    body_info.joint_position_left_hand.y  - body_info.joint_position_left_elbow.y ) * 180/PI;

    if(angle_r<0){
      angle_r  = angle_r + 360;
    }
    if(angle_l<0){
      angle_l = angle_l + 360;
    }
    printf("Right hand angle is: %3.0f \n", angle_r);
    printf("Left hand angle is: %3.0f\n",  angle_l);


    if((angle_r < 90 + TOLERANCE) && (angle_r > 90 - TOLERANCE)){
      printf("Right Hand is up at: %3.0f degrees\n", angle_r);
    }
    if((angle_l < 90 + TOLERANCE) && (angle_l > 90 - TOLERANCE)){
      printf("Left Hand is up at: %3.0f degrees\n", angle_l);
    }

    if((angle_r < 270 + TOLERANCE) && (angle_r > 270 - TOLERANCE)){
      printf("Right Hand is down at: %3.0f degrees\n", angle_r);
    }
    if((angle_l < 270 + TOLERANCE) && (angle_l > 270 - TOLERANCE)){
      printf("Left Hand is down at: %3.0f degrees\n", angle_l);
    }

    if((angle_r > 0) && (angle_r < 0 + TOLERANCE) || (angle_r<360)&&(angle_r > 360 - TOLERANCE)){
      printf("Right Hand is to the side at: %3.0f degrees\n", angle_r);
    }
    if((angle_l < 180 + TOLERANCE) && (angle_l > 180 - TOLERANCE)){
      printf("Left Hand is to the side at: %3.0f degrees\n", angle_l);
    }

    printf("----------------------------\n\n");

  }
}



void PublishMarker(int id, float x, float y, float z, float color_r, float color_g, float color_b)
{
  // Display marker for RVIZ to show where robot thinks person is
  // For Markers info, see http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

  // ROS_INFO("DBG: PublishMarker called");
  //if( id != 1)
  //  printf ("DBG PublishMarker called for %f, %f, %f\n", x,y,z);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "astra_camera_link"; // "base_link";
  marker.header.stamp = ros::Time::now();

  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "astra_body_tracker";
  marker.id = id; // This must be id unique for each marker

  uint32_t shape = visualization_msgs::Marker::SPHERE;
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = color_r;
  marker.color.g = color_g;
  marker.color.b = color_b;
  marker.color.a = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1; // size of marker in meters
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;


  //ROS_INFO("DBG: Publishing Marker");
  marker_pub_.publish(marker);

}


void runSimon()
{
    time_start = clock();
    set_key_handler();
    astra_initialize();
    const char* licenseString = "<INSERT LICENSE KEY HERE>";
    orbbec_body_tracking_set_license(licenseString);

    astra_streamsetconnection_t sensor;
    astra_streamset_open("device/default", &sensor);

    astra_reader_t reader;
    astra_reader_create(sensor, &reader);

    astra_bodystream_t bodyStream;
    astra_reader_get_bodystream(reader, &bodyStream);

    astra_stream_start(bodyStream);
    bool gamestart = false;
    do{
        astra_update();

        astra_reader_frame_t frame;
        astra_status_t rc = astra_reader_open_frame(reader, 0, &frame);

        if (rc == ASTRA_STATUS_SUCCESS){
            astra_bodyframe_t bodyFrame;
            astra_frame_get_bodyframe(frame, &bodyFrame);

            astra_frame_index_t frameIndex;
            astra_bodyframe_get_frameindex(bodyFrame, &frameIndex);

            output_bodies(bodyFrame);

            astra_reader_close_frame(&frame);

            clock_t time_now = clock();
            clock_t stopwatch = time_now - time_start;
            int stopwatchint = (int)stopwatch/CLOCKS_PER_SEC;
            printf("time is %d and status is %d\n",stopwatchint,  gamestart);

            // 0 for up, 1 for down, 2 for out.
            // Left arm = index 0, Right arm = index 1
            if(actioncom == true){
                      do{
                          rand_hand = rand()%3;
                          rand_act = rand()%3;
                          memcpy(state_next,state_curr, sizeof(state_curr));
                          if(rand_hand < 2){
                            state_next[rand_hand] = rand_act;
                          }
                          else{
                            state_next[0] = rand_act;
                            state_next[1] = rand_act;
                          }
                      }while(state_curr[0] == state_next[0] && state_curr[1] == state_next[1]);

                        rand_says = rand()%10;

                        if(rand_says < 4){
                          simoncmd = 1; // Simon doesn't say.
                        }
                        else{
                          memcpy(state_curr,state_next, sizeof(state_curr));
                          simoncmd = 0; // Simon does say.
                        }
                        actioncom  = false;
                        actionissued = false;
                        gameround += 1;
            }

              if (stopwatchint>15 && stopwatchint<20 && gamestart == false){
                if (num_players == 0){
                  std::string noplayers = "No players detected, this game will end. good bye.";
                  txttospeak(noplayers);
                  break;
                }
                std::string gamestarttxt = "All players are accounted for. The game will start in 5 seconds.";
                txttospeak(gamestarttxt);
                gamestart = true;
                 time_start = clock();
              }

              if (stopwatchint>5 && stopwatchint<10 && actionissued == false && gamestart == true){
                std::string nxtcmd =  says[simoncmd] +" "+ hand[rand_hand] +" "+ action[rand_act];
                txttospeak(nxtcmd);
                actionissued = true;
              }

              if (stopwatchint>10 && stopwatchint<15 && actioncom == false && actionissued == true){
                action_check(bodyFrame, state_curr);
                time_start = clock();
                actioncom = true;
              }

              if(gameround == (maxrounds+1)){
                std::string gameend =  "The remaining players have won the game. Thank you for playing. Good bye";
                txttospeak(gameend);
                break;
              }
        }

         ros::spinOnce();  // ROS

    } while (shouldContinue);

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);

    astra_terminate();

}

void action_check(astra_bodyframe_t bodyFrame, int correct_state[2])
{
    int i;
    astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS){
        printf("Error %d in astra_bodyframe_body_list()\n", rc);
        return;
    }
    for(i = 0; i < bodyList.count; ++i){
        astra_body_t* body = &bodyList.bodies[i];

        astra_body_id_t bodyId = body->id;

        astra_body_status_t bodyStatus = body->status;

        const astra_vector3f_t* centerOfMass = &body->centerOfMass;
        const astra_body_tracking_feature_flags_t features = body->features;
        astra_joint_t* joint;  // AstraSDK/include/astra/capi/streams/body_types.h

        const bool jointTrackingEnabled =
          (features & ASTRA_BODY_TRACKING_JOINTS) == ASTRA_BODY_TRACKING_JOINTS;
        const bool handPoseRecognitionEnabled =
          (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

        geometry_msgs::PoseStamped body_pose;
        astra_body_tracker::BodyInfo_ <astra_body_tracker::BodyInfo> body_info;

        body_pose.header.frame_id = "astra_camera_link"; // "base_link";
        body_pose.header.stamp = ros::Time::now();

        body_info.body_id = bodyId;
        body_info.tracking_status = bodyStatus;

        joint = &body->joints[ASTRA_JOINT_LEFT_ELBOW];
        body_info.joint_position_left_elbow.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
        body_info.joint_position_left_elbow.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
        body_info.joint_position_left_elbow.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

        joint = &body->joints[ASTRA_JOINT_LEFT_HAND];
        body_info.joint_position_left_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
        body_info.joint_position_left_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
        body_info.joint_position_left_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;


        joint = &body->joints[ASTRA_JOINT_RIGHT_ELBOW];
        body_info.joint_position_right_elbow.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
        body_info.joint_position_right_elbow.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
        body_info.joint_position_right_elbow.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

        joint = &body->joints[ASTRA_JOINT_RIGHT_HAND];
        body_info.joint_position_right_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
        body_info.joint_position_right_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
        body_info.joint_position_right_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

        ////////////////////////////////////////////////////
        // Publish Body Data
        body_tracking_pose_pub_.publish(body_pose); // this is position only!
        body_tracking_data_pub_.publish(body_info);
        ////////////////////////////////////////////////////

        PublishMarker(
          ASTRA_JOINT_RIGHT_HAND, // ID
          body_info.joint_position_right_hand.x,
          body_info.joint_position_right_hand.y,
          body_info.joint_position_right_hand.z,
          1.0, 0.0, 0.0 ); // r,g,b

        PublishMarker(
          ASTRA_JOINT_RIGHT_ELBOW, // ID
          body_info.joint_position_right_elbow.x,
          body_info.joint_position_right_elbow.y,
          body_info.joint_position_right_elbow.z,
          1.0, 0.0, 0.0 ); // r,g,b

        PublishMarker(
          ASTRA_JOINT_LEFT_HAND, // ID
          body_info.joint_position_left_hand.x,
          body_info.joint_position_left_hand.y,
          body_info.joint_position_left_hand.z,
          1.0, 0.0, 0.0 ); // r,g,b

        PublishMarker(
          ASTRA_JOINT_LEFT_ELBOW, // ID
          body_info.joint_position_left_elbow.x,
          body_info.joint_position_left_elbow.y,
          body_info.joint_position_left_elbow.z,
          1.0, 0.0, 0.0 ); // r,g,b

        //m to inch 39.3

          float angle_r = atan2(body_info.joint_position_right_hand.z - body_info.joint_position_right_elbow.z,\
             body_info.joint_position_right_hand.y  - body_info.joint_position_right_elbow.y ) * 180/PI;
          float angle_l = atan2(body_info.joint_position_left_hand.z - body_info.joint_position_left_elbow.z, \
            body_info.joint_position_left_hand.y  - body_info.joint_position_left_elbow.y ) * 180/PI;

          int user_state[] = {-1,-1};
          if(angle_r<0){
            angle_r  = angle_r + 360;
          }
          if(angle_l<0){
            angle_l = angle_l + 360;
          }

          if((angle_r < 90 + TOLERANCE) && (angle_r > 90 - TOLERANCE)){
            printf("Right Hand is up at: %3.0f degrees\n", angle_r);
            user_state[1] = 0;
          }
          if((angle_l < 90 + TOLERANCE) && (angle_l > 90 - TOLERANCE)){
            printf("Left Hand is up at: %3.0f degrees\n", angle_l);
            user_state[0] = 0;
          }

          if((angle_r < 270 + TOLERANCE) && (angle_r > 270 - TOLERANCE)){
            printf("Right Hand is down at: %3.0f degrees\n", angle_r);
            user_state[1] = 1;
          }
          if((angle_l < 270 + TOLERANCE) && (angle_l > 270 - TOLERANCE)){
            printf("Left Hand is down at: %3.0f degrees\n", angle_l);
            user_state[0] = 1;
          }

          if((angle_r > 0) && (angle_r < 0 + TOLERANCE) || \
          (angle_r<360)&&(angle_r > 360 - TOLERANCE)){
            printf("Right Hand is to the side at: %3.0f degrees\n", angle_r);
            user_state[1] = 2;
          }
          if((angle_l < 180 + TOLERANCE) && (angle_l > 180 - TOLERANCE)){
            printf("Left Hand is to the side at: %3.0f degrees\n", angle_l);
            user_state[0] = 2;
          }

          int playerNo = find_player_no(bodyId);
          if(user_state[0] != correct_state[0] || user_state[1] != correct_state[1] ){
            std::string failcmd = "Player "+ ToString(playerNo) + " you have failed. Please get out.";
            txttospeak(failcmd);
          }
          else{
            std::string passcmd = "Player "+ ToString(playerNo) + " you have succeeded.";
            txttospeak(passcmd);
          }
    }
}





//////////////////////////////////////////////////////////
private:
  std::string _name;
  ros::NodeHandle nh_;
  //ros::Subscriber robot_behavior_state_;
  std::string myparm1_;

  //ros::Publisher body_tracking_status_pub_;
  ros::Publisher body_tracking_pose_pub_;
  ros::Publisher body_tracking_data_pub_;
  ros::Publisher marker_pub_;

};



// The main entry point for this node.
int main( int argc, char *argv[]){
srand(time(NULL));
  ros::init( argc, argv, "astra_body_tracker" );
  astra_body_tracker_node node(ros::this_node::getName());
  std::string welcome = "Welcome to Simon Says.";
  txttospeak (welcome);
  std::string initpos = "Stand in front of the camera and place your hands down to your sides.";
  txttospeak(initpos);

  node.runSimon();
  //ros::spin();

  return 0;
}


int find_player_no(int body_id){
  for (int j = 0; j < num_players; j++){
    if(playerId[j] == body_id){
      return j+1;
    }
  }
  return 0;
}

void txttospeak( std::string text){
  std::string speakstr = "espeak '"+text+"'";
  const char * speak = speakstr.c_str();
  int isys = system(speak);
}
