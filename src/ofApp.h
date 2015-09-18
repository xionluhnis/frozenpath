#pragma once

#undef Success
#undef Status

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "io/openni2_grabber.h"

#include "ofMain.h"
#include "camera/ofStableCam.h"
#include "ui/ofButton.h"

#include <vector>

#undef Success
#undef Status

#define CAPTURE_MEMORY 	5

typedef boost::shared_ptr<ofMesh> ofMeshPtr;

namespace pcl {
		class Grabber;
}

enum Action {
		Capture = 0,
		Running = 1,
		Unknown = 2
};

class ofApp : public ofBaseApp{
	public:
		void setup();
		void update();
		void draw();
		void updateCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

    private:
        // World objects
        ofStableCam     	cam;
        ofLight         	light;
        
        // Point Cloud
        pcl::Grabber*			pcGrabber;
        ofMeshPtr					pointCloud[CAPTURE_MEMORY];
        float 						minZ, maxZ;
        unsigned int 			captured;
        
        // UI
        std::vector<ofButton> ui;
};
